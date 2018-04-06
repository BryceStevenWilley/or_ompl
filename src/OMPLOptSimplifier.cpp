/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#include <time.h>
#include <tinyxml.h>
#include <iostream>
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/scope_exit.hpp>
#include <ompl/config.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/CollisionEvaluator.h>
#include <ompl/base/objectives/ObstacleConstraint.h>
#include <ompl/base/objectives/JointDistanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/trajopt/TrajOpt.h>
#include <ompl/trajopt/modeling.h>
#include <ompl/trajopt/optimizers.h>
#include <ompl/util/Time.h>

#include <or_ompl/config.h>
#include <or_ompl/OMPLConversions.h>
#include <or_ompl/OMPLPlanner.h>
#include <or_ompl/TSRGoal.h>
#include <or_ompl/SimplifierRegistry.h>
#include <or_ompl/OMPLOptSimplifier.h>

namespace or_ompl {

Eigen::MatrixXd TrajOptWrapper::jacobianAtPoint(ompl::base::CollisionInfo info, int which)
{
    rad_->SetDOFValues(info.x);
    std::string link_name = info.link_names[which];
    OpenRAVE::Vector stupid_vector(info.points[which].x(), info.points[which].y(), info.points[which].z());
    return rad_->PositionJacobian(link2index_[link_name], stupid_vector);
}

bool TrajOptWrapper::extraCollisionInformation(std::vector<double> configuration, 
                                          std::vector<ompl::base::CollisionInfo>& collisionStructs)
{
    // TODO: worry about collision caching later.
    rad_->SetDOFValues(configuration);
    std::vector<trajopt::Collision> collisions;
    coll_check_->LinksVsAll(links_ , collisions, -1);
    for (auto coll : collisions)
    {
        ompl::base::CollisionInfo collisionStruct;
        collisionStruct.x = configuration;

        Eigen::Vector3d ptA(coll.ptA.x, coll.ptA.y, coll.ptA.z);   
        Eigen::Vector3d ptB(coll.ptB.x, coll.ptB.y, coll.ptB.z);   
        Eigen::Vector3d normal(coll.normalB2A.x, coll.normalB2A.y, coll.normalB2A.z);

        auto itA = link2index_.find(coll.linkA->GetName());
        auto itB = link2index_.find(coll.linkB->GetName());
        collisionStruct.signedDist = coll.distance; 
        // If both exist, it's a self collision.
        if (itA != link2index_.end() && itB != link2index_.end())
        {
            collisionStruct.points.push_back(ptA);
            collisionStruct.points.push_back(ptB);
            collisionStruct.link_names.push_back(coll.linkA->GetName());
            collisionStruct.link_names.push_back(coll.linkB->GetName());
            collisionStruct.normal = normal;
        }
        // Otherwise, only link is in collision with the env.
        else if (itA != link2index_.end())
        {
            collisionStruct.points.push_back(ptA);
            collisionStruct.link_names.push_back(coll.linkA->GetName());
            collisionStruct.normal = normal;
        }
        else if (itB != link2index_.end())
        {
            collisionStruct.points.push_back(ptB);
            collisionStruct.link_names.push_back(coll.linkB->GetName());
            collisionStruct.normal = -normal;
        }
        // Otherwise, two world objects must be in collision with one another.
        else
        {
            continue;
        }
        collisionStructs.push_back(collisionStruct);
    }
    return collisionStructs.size() > 0;
}

bool TrajOptWrapper::extraCollisionInformation(std::vector<double> configuration0, std::vector<double> configuration1,
                                               std::vector<ompl::base::ContinuousCollisionInfo>& collisionStructs)
{
    // TODO: worry about collision caching.
    rad_->SetDOFValues(configuration0);
    std::vector<trajopt::Collision> collisions;
    coll_check_->CastVsAll(*rad_, links_, configuration0, configuration1, collisions);
    for (auto coll : collisions)
    {
        ompl::base::ContinuousCollisionInfo collisionStruct;
        collisionStruct.x_0 = configuration0;
        collisionStruct.x_1 = configuration1;

        Eigen::Vector3d ptA(coll.ptA.x, coll.ptA.y, coll.ptA.z);
        Eigen::Vector3d ptB(coll.ptB.x, coll.ptB.y, coll.ptB.z);
        Eigen::Vector3d normal(coll.normalB2A.x, coll.normalB2A.y, coll.normalB2A.z);
        auto itA = link2index_.find(coll.linkA->GetName());
        auto itB = link2index_.find(coll.linkB->GetName());
        collisionStruct.signedDist = coll.distance; 
        // If both links exist, it's a self collision.
        if (itA != link2index_.end() && itB != link2index_.end())
        {
            // TODO: make continuous collisions handle self collisions.
        }
        // Otherwise, only one link is in collision with the env.
        else if (itA != link2index_.end())
        {
            collisionStruct.p0 = ptA;
            // TODO: according to trajopt, this shouldn't happen, there's only a spot for p1 in B.
            RAVELOG_ERROR("WARNING: possibly putting the wrong point in a struct: bad assumption in TrajOpt.");
            Eigen::Vector3d ptB1(coll.ptB1.x, coll.ptB1.y, coll.ptB1.z);
            collisionStruct.p1 = ptB1;
            collisionStruct.link_name = coll.linkA->GetName();
            collisionStruct.normal = normal;
            collisionStruct.alpha = coll.time;
        }
        else if (itB != link2index_.end())
        {
            collisionStruct.p0 = ptB;
            Eigen::Vector3d ptB1(coll.ptB1.x, coll.ptB1.y, coll.ptB1.z);
            collisionStruct.p1 = ptB1;
            collisionStruct.link_name = coll.linkB->GetName();
            collisionStruct.normal = -normal;
            collisionStruct.alpha = coll.time;
        }
        // Otherwise, two world objects must be in collision with one another.
        else
        {
            continue;
        }
        collisionStructs.push_back(collisionStruct);
    }
    return collisionStructs.size() > 0;
}

OMPLOptSimplifier::OMPLOptSimplifier(OpenRAVE::EnvironmentBasePtr penv,
                         PlannerFactory const &simplifier_factory)
    : AOMPLPlanner(penv, simplifier_factory) {

    RegisterCommand("GetCost",
        boost::bind(&OMPLOptSimplifier::GetCost,this,_1,_2),
        "get cost information for the given trajectory");
}

OMPLOptSimplifier::~OMPLOptSimplifier() {
}

bool OMPLOptSimplifier::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input) {
    OMPLPlannerParametersPtr params = boost::make_shared<OMPLPlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

bool OMPLOptSimplifier::InitPlan(OpenRAVE::RobotBasePtr robot,
                           PlannerParametersConstPtr params_raw) {
    m_initialized = false;

    try {
        typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

        if (!robot) {
            RAVELOG_ERROR("Robot must not be NULL.\n");
            return false;
        } else if (!params_raw) {
            RAVELOG_ERROR("Parameters must not be NULL.\n");
            return false;
        }

        m_robot = robot;
        m_totalPlanningTime = 0.0;

        std::vector<int> dof_indices = robot->GetActiveDOFIndices();
        const unsigned int num_dof = dof_indices.size();
        m_parameters = boost::make_shared<OMPLPlannerParameters>();
        m_parameters->copy(params_raw);

        RAVELOG_DEBUG("Creating state space.\n");
        m_state_space = CreateStateSpace(robot, *m_parameters);
        if (!m_state_space) {
            RAVELOG_ERROR("Failed creating state space.\n");
            return false;
        }

        RAVELOG_DEBUG("Creating OMPL setup.\n");
        m_simple_setup.reset(new ompl::geometric::SimpleSetup(m_state_space));

        RAVELOG_DEBUG("Setting state validity checker.\n");
        if (m_state_space->isCompound()) {
            m_or_validity_checker.reset(new OrStateValidityChecker(
                m_simple_setup->getSpaceInformation(), m_robot, dof_indices, m_parameters->m_doBaked));
        } else {
            m_or_validity_checker.reset(new RealVectorOrStateValidityChecker(
                m_simple_setup->getSpaceInformation(), m_robot, dof_indices, m_parameters->m_doBaked));
        }
#ifdef OR_OMPL_HAS_BOOSTSMARTPTRS
        m_simple_setup->setStateValidityChecker(
            boost::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#else
        m_simple_setup->setStateValidityChecker(
            std::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#endif

        // start validity checker
        m_or_validity_checker->start();
        BOOST_SCOPE_EXIT((m_or_validity_checker)) {
            m_or_validity_checker->stop();
        } BOOST_SCOPE_EXIT_END

        RAVELOG_DEBUG("Setting initial configuration.\n");
        if (m_parameters->vinitialconfig.size() % num_dof != 0) {
            RAVELOG_ERROR("Start configuration has incorrect DOF;"
                          " expected multiple of %d, got %d.\n",
                          num_dof, m_parameters->vinitialconfig.size());
            return false;
        }
        unsigned int num_starts = m_parameters->vinitialconfig.size() / num_dof;
        if (num_starts == 0) {
            RAVELOG_ERROR("No initial configurations provided.\n");
            return false;
        }

        if (num_starts == 1) {
            ScopedState q_start(m_state_space);
            for (size_t i = 0; i < num_dof; i++) {
                q_start[i] = m_parameters->vinitialconfig[i];
            }
            if (!m_or_validity_checker->isValid(q_start.get())) {
                RAVELOG_ERROR("Single initial configuration in collision.\n");
                return false;
            }
        }

        for (unsigned int istart=0; istart<num_starts; istart++)
        {
            ScopedState q_start(m_state_space);
            for (size_t i = 0; i < num_dof; i++) {
                q_start[i] = m_parameters->vinitialconfig[istart*num_dof + i];
            }
            m_simple_setup->addStartState(q_start);
        }

        RAVELOG_DEBUG("Setting goal configuration.\n");
        std::vector<TSRChain::Ptr> goal_chains;
        BOOST_FOREACH(TSRChain::Ptr tsr_chain, m_parameters->m_tsrchains){
            if (tsr_chain->sampleGoal()){
                tsr_chain->setEnv(robot->GetEnv()); // required to enable distance to TSR chains
                goal_chains.push_back(tsr_chain);
            } else {
                RAVELOG_ERROR("Only goal TSR chains are supported by OMPL. Failing.\n");
                return false;
            }
        }

        if (goal_chains.size() > 0 && m_parameters->vgoalconfig.size() > 0){
            RAVELOG_ERROR("A goal TSR chain has been supplied and a goal configuration"
                          " has been specified. The desired behavior is ambiguous."
                          " Please specified one or the other.\n");
            return false;
        }

        if (goal_chains.size() > 0) {
            TSRGoal::Ptr goaltsr(new TSRGoal(m_simple_setup->getSpaceInformation(),
                                             goal_chains,
                                             robot,
                                             m_or_validity_checker));
            m_simple_setup->setGoal(goaltsr);
        } else {
            if (m_parameters->vgoalconfig.size() % num_dof != 0) {
                RAVELOG_ERROR("End configuration has incorrect DOF;"
                              "  expected multiple of %d, got %d.\n",
                              num_dof, m_parameters->vgoalconfig.size());
                return false;
            }
            unsigned int num_goals = m_parameters->vgoalconfig.size() / num_dof;
            if (num_goals == 0) {
                RAVELOG_ERROR("No goal configurations provided.\n");
                return false;
            }

            if (num_goals == 1) {
                ScopedState q_goal(m_state_space);
                for (size_t i = 0; i < num_dof; i++) {
                    q_goal[i] = m_parameters->vgoalconfig[i];
                }

                if (!m_or_validity_checker->isValid(q_goal.get())) {
                    RAVELOG_ERROR("Single goal configuration is in collision.\n");
                    return false;
                }

                m_simple_setup->setGoalState(q_goal);
            } else {
                // if multiple possible goals specified,
                // don't check them all (this might be expensive)
                // and instead lead the planner check some
                ompl::base::GoalPtr ompl_goals(new ompl::base::GoalStates(
                    m_simple_setup->getSpaceInformation()));
                for (unsigned int igoal=0; igoal<num_goals; igoal++)
                {
                    ScopedState q_goal(m_state_space);
                    for (size_t i = 0; i < num_dof; i++) {
                        q_goal[i] = m_parameters->vgoalconfig[igoal*num_dof + i];
                    }
                    ompl_goals->as<ompl::base::GoalStates>()->addState(q_goal);
                }
                m_simple_setup->setGoal(ompl_goals);
            }
        }

        RAVELOG_DEBUG("Creating planner.\n");
        m_planner = CreatePlanner(*m_parameters);
        if (!m_planner) {
            RAVELOG_ERROR("Failed creating OMPL planner.\n");
            return false;
        }
        m_simple_setup->setPlanner(m_planner);

        if (m_planner->getName() == "TrajOpt")
        {
            auto params_map = GetParameterVector(*m_parameters);
            bool continuous_collisions = true;
            if (params_map.count("continuous_collisions") != 0)
            {
               continuous_collisions = (params_map["continuous_collisions"] == "true") ? true : false; 
            }
            double safety_distance = 0.025;
            if (params_map.count("safety_distance") != 0)
            {
                safety_distance = atof(params_map["safety_distance"].c_str());
            }
            const ompl::base::SpaceInformationPtr &si = m_simple_setup->getSpaceInformation();
            auto bare_bones = std::make_shared<ompl::base::MultiConvexifiableOptimization>(si);
            bare_bones->addObjective(std::make_shared<ompl::base::JointDistanceObjective>(si));
    
            // Obstacle Objective: Make a jacobian and a Collision Info getter base on TrajOpt.
            trajopt::Configuration *rad = new trajopt::RobotAndDOF(m_robot, robot->GetActiveDOFIndices());
            trajopt::CollisionCheckerPtr coll_check =
                    trajopt::CollisionChecker::GetOrCreate(*m_robot->GetEnv());
            m_wrapper = std::make_shared<TrajOptWrapper>(rad, coll_check);
            ompl::base::JacobianFn jacobian = [this](ompl::base::CollisionInfo collisionStruct, int which) {
                return this->m_wrapper->jacobianAtPoint(collisionStruct, which);
            };
            if (continuous_collisions)
            {
                RAVELOG_DEBUG("Created a TrajOpt planner, adding continuous convex cost.");

                ompl::base::WorkspaceContinuousCollisionFn collisions = [this](std::vector<double> configuration0, std::vector<double> configuration1,
                                                                     std::vector<ompl::base::ContinuousCollisionInfo>& collisionStructs) {
                    return this->m_wrapper->extraCollisionInformation(configuration0, configuration1, collisionStructs);
                };
                bare_bones->addObjective(std::make_shared<ompl::base::ObstacleConstraint>(si, safety_distance, collisions, jacobian));
            }
            else
            {
                RAVELOG_DEBUG("Created a TrajOpt planner, adding discrete convex cost.");
    
                ompl::base::WorkspaceCollisionFn collisions = [this](std::vector<double> configuration,
                                                                     std::vector<ompl::base::CollisionInfo>& collisionStructs) {
                    return this->m_wrapper->extraCollisionInformation(configuration, collisionStructs);
                };
                bare_bones->addObjective(std::make_shared<ompl::base::ObstacleConstraint>(si, safety_distance, collisions, jacobian));
            }
            m_simple_setup->setOptimizationObjective(bare_bones);
        }
        else
        {
            RAVELOG_DEBUG("Did not reate a TrajOpt planner.");
        }

        m_initialized = true;
        return true;
    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("InitPlan failed: %s\n", e.what());
        return false;
    }
}

OpenRAVE::PlannerStatus OMPLOptSimplifier::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    // Needed so TrajOpt can init the problem instance.
    m_simple_setup->setup();
    if (!m_initialized)
    {
        RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n");
        return OpenRAVE::PS_Failed;
    }
    else if (ptraj && ptraj->GetNumWaypoints() != 0)
    {
        RAVELOG_WARN("Using existing path!");
        // Start off with existing path found by something else.
        ompl::geometric::PathGeometric path(m_simple_setup->getSpaceInformation());
        FromORTrajectory(m_robot, ptraj, path);
        m_planner->as<ompl::geometric::TrajOpt>()->setInitialTrajectory(path);
    } else
    {
        RAVELOG_WARN("ptraj is not initialized?");
    }

    boost::chrono::steady_clock::time_point const tic
       = boost::chrono::steady_clock::now();

    OpenRAVE::PlannerStatus planner_status;
    try {
        // TODO: Configure anytime algorithms to keep planning.
        //m_simpleSetup->getGoal()->setMaximumPathLength(0.0);

        // start validity checker
        m_or_validity_checker->start();
        BOOST_SCOPE_EXIT((m_or_validity_checker)) {
            m_or_validity_checker->stop();
        } BOOST_SCOPE_EXIT_END

        ompl::base::PlannerStatus ompl_status;

        if (m_parameters->m_dat_filename != "")
        {
            std::ofstream file_out;
            file_out.open(m_parameters->m_dat_filename);
            file_out << "[";

            auto start_time_point = ompl::time::now();
            auto callback = [&file_out, start_time_point, this](sco::OptProb *prob, std::vector<double> &x, double cost) {
                static int iteration = 0;
                iteration++;
                if (iteration == 1)
                {
                file_out << "{\"iter\":" << iteration << 
                            ", \"seconds_elapsed\":" << ompl::time::seconds(ompl::time::now() - start_time_point) <<
                            ", \"cost\":" << cost << "}";
                }
                else
                {
                    file_out << ",{\"iter\":" << iteration << 
                                ", \"seconds_elapsed\":" << ompl::time::seconds(ompl::time::now() - start_time_point) <<
                                ", \"cost\":" << cost << "}";
                }
            };
            m_planner->as<ompl::geometric::TrajOpt>()->setOptimizerCallback(callback);
            ompl_status = m_simple_setup->solve(m_parameters->m_timeLimit);
            file_out << "]";
            file_out.close();
        }
        else
        {
            ompl_status = m_simple_setup->solve(m_parameters->m_timeLimit);
        }

        // Handle OMPL return codes, set planner_status and ptraj
        if (ompl_status == ompl::base::PlannerStatus::EXACT_SOLUTION
            || ompl_status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {

            if (m_simple_setup->haveSolutionPath()) {
                ompl::geometric::PathGeometric path = m_simple_setup->getSolutionPath();
                RAVELOG_DEBUG("Path length: %.3f, smoothness, %.3f\n", path.length(), path.smoothness());
                ToORTrajectory(m_robot, m_simple_setup->getSolutionPath(), ptraj);
                if (ompl_status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
                    planner_status = OpenRAVE::PS_HasSolution;
                } else {
                    planner_status = OpenRAVE::PS_InterruptedWithSolution;
                }
            } else {
                RAVELOG_ERROR("Planner returned %s, but no path found!\n", ompl_status.asString().c_str());
                planner_status = OpenRAVE::PS_Failed;
            }

        } else {
            // Intended to handle:
            // - PlannerStatus::INVALID_START
            // - PlannerStatus::INVALID_GOAL
            // - PlannerStatus::UNRECOGNIZED_GOAL_TYPE
            // - PlannerStatus::CRASH
            // - PlannerStatus::ABORT
            // - PlannerStatus::TIMEOUT
            // (these cases are not handled explicitly because different versions
            //  of OMPL support different error cases)
            RAVELOG_ERROR("Planner returned %s.\n", ompl_status.asString().c_str());
            planner_status = OpenRAVE::PS_Failed;
        }

    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("Planning failed: %s\n", e.what());
        planner_status = OpenRAVE::PS_Failed;
    }

    boost::chrono::steady_clock::time_point const toc
        = boost::chrono::steady_clock::now();
    m_totalPlanningTime += boost::chrono::duration_cast<
        boost::chrono::duration<double> >(toc - tic).count();

    return planner_status;
}

bool OMPLOptSimplifier::GetCost(std::ostream & sout, std::istream &sin) const
{
    OpenRAVE::TrajectoryBasePtr traj = RaveCreateTrajectory(m_robot->GetEnv());
    traj->deserialize(sin);
    ompl::geometric::PathGeometric path(m_simple_setup->getSpaceInformation());
    FromORTrajectory(m_robot, traj, path);
    // Now, get the cost of 'path'.
    if (m_planner->getName() == "TrajOpt")
    {
        path.interpolate(m_planner->as<ompl::geometric::TrajOpt>()->getTimeStepCount());
    }
    ompl::base::OptimizationObjectivePtr opt_obj = m_simple_setup->getOptimizationObjective();
    if (opt_obj == nullptr)
    {
        opt_obj = std::make_shared<ompl::base::PathLengthOptimizationObjective>(m_simple_setup->getSpaceInformation());
        // Still not there? Return false.
        if (opt_obj == nullptr)
            return false;
    }
    ompl::base::Cost cost = path.cost(opt_obj); 
    sout << cost.value();
    return true;
}

} // namespace or_ompl
