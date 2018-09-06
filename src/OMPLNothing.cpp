/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

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

/*********************************
 * A null pattern path simplifier.
 * Author: Bryce Willey
 *********************************/


#include <boost/make_shared.hpp>
#include <boost/scope_exit.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Time.h>
#include <ompl/tools/lightning/DynamicTimeWarp.h>

#include <or_ompl/config.h>
#include <or_ompl/OMPLConversions.h>
#include <or_ompl/OMPLNothing.h>

using OpenRAVE::PA_None;
using OpenRAVE::PA_Interrupt;
using OpenRAVE::PA_ReturnWithAnySolution;

using OpenRAVE::PS_HasSolution;
using OpenRAVE::PS_InterruptedWithSolution;

namespace or_ompl {

OMPLNothing::OMPLNothing(OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::PlannerBase(env) {
        RegisterCommand("GetDTW",
            boost::bind(&OMPLNothing::GetDTW, this, _1, _2),
            "get the dynamic time warping distance between two paths");
        RegisterCommand("GetSmoothness",
            boost::bind(&OMPLNothing::GetSmoothness, this, _1, _2),
            "get the OMPL smoothness of the entire path");
        RegisterCommand("GetClearance",
            boost::bind(&OMPLNothing::GetClearance, this, _1, _2),
            "get the clearance of the entire path");
}

OMPLNothing::~OMPLNothing() {
}

bool OMPLNothing::InitPlan(OpenRAVE::RobotBasePtr robot,
                              PlannerParametersConstPtr params_raw) {
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return false;
    } else if (!params_raw) {
        RAVELOG_ERROR("Parameters must not be NULL.\n");
        return false;
    }

    m_robot = robot;
    m_cspec = m_robot->GetActiveConfigurationSpecification();
    std::vector<int> dof_indices = robot->GetActiveDOFIndices();

    m_parameters = boost::make_shared<OMPLPlannerParameters>();
    m_parameters->copy(params_raw);

    try {
        using ompl::base::SpaceInformation;
        using ompl::geometric::PathSimplifier;

        m_state_space = CreateStateSpace(robot, *m_parameters);
        m_space_info.reset(new SpaceInformation(m_state_space));
        if (m_state_space->isCompound()) {
            m_or_validity_checker.reset(new OrStateValidityChecker(
                m_space_info, m_robot, dof_indices, m_parameters->m_doBaked));
        } else {
            m_or_validity_checker.reset(new RealVectorOrStateValidityChecker(
                m_space_info, m_robot, dof_indices, m_parameters->m_doBaked));
        }
#ifdef OR_OMPL_HAS_BOOSTSMARTPTRS
        m_space_info->setStateValidityChecker(
            boost::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#else
        m_space_info->setStateValidityChecker(
            std::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#endif

        m_space_info->setup();
        m_simplifier.reset(new PathSimplifier(m_space_info));
        return true;
    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("IntPlan failed: %s\n", e.what());
        return false;
    }
}

bool OMPLNothing::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input) {
    OMPLPlannerParametersPtr params = boost::make_shared<OMPLPlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

OpenRAVE::PlannerStatus OMPLNothing::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

    if (!m_simplifier) {
        RAVELOG_ERROR("Not initialized. Did you call InitPlan?\n");
        return OpenRAVE::PS_Failed;
    } else if (!ptraj) {
        RAVELOG_ERROR("Input trajectory is NULL.\n");
        return OpenRAVE::PS_Failed;
    } else if (m_parameters->m_timeLimit <= 0) {
        RAVELOG_ERROR("Time limit must be positive; got %f.",
                      m_parameters->m_timeLimit);
        return OpenRAVE::PS_Failed;
    } else if (ptraj->GetNumWaypoints() == 0) {
        RAVELOG_WARN("Input trajectory is empty; there is nothing to do.\n");
        return OpenRAVE::PS_HasSolution;
    }

    // Convert the OpenRAVE trajectory into an OMPL path.
    BOOST_ASSERT(m_space_info);
    ompl::geometric::PathGeometric path(m_space_info);
    size_t const num_dof = m_cspec.GetDOF();

    RAVELOG_DEBUG("Create OMPL path with %d DOF and %d waypoints.\n",
                  num_dof, ptraj->GetNumWaypoints());

    for (size_t iwaypoint = 0; iwaypoint < ptraj->GetNumWaypoints(); ++iwaypoint) {
        // Extract the OpenRAVE waypoint. Default to the current configuration
        // for any missing DOFs.
        std::vector<OpenRAVE::dReal> waypoint_openrave;
        m_robot->GetActiveDOFValues(waypoint_openrave);
        ptraj->GetWaypoint(iwaypoint, waypoint_openrave, m_cspec);

        // Insert the waypoint into the OMPL path.
        ScopedState waypoint_ompl(m_space_info);
        for (size_t idof = 0; idof < num_dof; ++idof) {
            waypoint_ompl[idof] = waypoint_openrave[idof];
        }
        path.append(waypoint_ompl.get());
    }

    double const length_before = path.length();
    double const smoothness_before = path.smoothness();

    RAVELOG_WARN(
        "Path length: %.3f, "
        "Smoothness from %.3f.\n",
        length_before,
        smoothness_before
    );

    return PS_HasSolution;
}

void OMPLNothing::readPathGeometric(ompl::geometric::PathGeometric& path, std::istream &sin) const
{
    size_t MAX_STR_SIZE = 250000;
    OpenRAVE::TrajectoryBasePtr traj = RaveCreateTrajectory(m_robot->GetEnv());
    char *s = (char *)malloc(sizeof(char) * (MAX_STR_SIZE + 1));
    sin.getline(s, MAX_STR_SIZE);
    std::stringbuf sb(s);
    std::istream s_traj(&sb);
    traj->deserialize(s_traj);

    FromORTrajectory(m_robot, traj, path);
    free(s);
    return;
}

bool OMPLNothing::GetDTW(std::ostream &sout, std::istream &sin) const
{
    ompl::geometric::PathGeometric path1(m_space_info);
    ompl::geometric::PathGeometric path2(m_space_info);
    readPathGeometric(path1, sin);
    readPathGeometric(path2, sin);

    ompl::tools::DynamicTimeWarp dtw(m_space_info);
    double dist = dtw.getPathsScore(path1, path2);
    sout << dist;
    return true;
}

bool OMPLNothing::GetSmoothness(std::ostream &sout, std::istream &sin) const
{
    ompl::geometric::PathGeometric path(m_space_info);
    readPathGeometric(path, sin);

    double smoothness = path.smoothness();
    sout << smoothness;
    return true;
}

bool OMPLNothing::GetClearance(std::ostream &sout, std::istream &sin) const
{
    ompl::geometric::PathGeometric path(m_space_info);
    readPathGeometric(path, sin);

    // Interpolated to check as detailedly as possible.
    // TODO(brycew): maybe reconsider this?
    //path.interpolate();
    double clearance = path.clearance();
    sout << clearance;
    return true;
}

} // namespace or_ompl
