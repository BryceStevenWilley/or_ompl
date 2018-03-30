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

#ifndef OR_OMPL_OMPLOPTSIMPLIFIER_H_
#define OR_OMPL_OMPLOPTSIMPLIFIER_H_

#include <openrave-core.h>
#include <openrave/planner.h>
#include <openrave/planningutils.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/CollisionEvaluator.h>
#include <ompl/base/objectives/ObstacleConstraint.h>
#include <ompl/base/objectives/JointDistanceObjective.h>
#include <ompl/trajopt/modeling.h>

#include <or_ompl/StateSpaces.h>
#include <or_ompl/OMPLPlannerParameters.h>
#include <or_ompl/TrajOpt/configuration_space.hpp>
#include <or_ompl/TrajOpt/collision_checker.hpp>
#include <or_ompl/OMPLPlanner.h>

namespace or_ompl {

class TrajOptWrapper {
public:
    TrajOptWrapper(trajopt::Configuration *rad, trajopt::CollisionCheckerPtr coll_check) :
        rad_(rad), coll_check_(coll_check)
    {
        // Map the active link to its index in the robot.
        std::vector<int> indices;
        rad_->GetAffectedLinks(links_, true, indices);
        for (int i = 0; i < links_.size(); i++)
        {
            link2index_[links_[i]->GetName()] = indices[i];
        }
    }

    Eigen::MatrixXd jacobianAtPoint(ompl::base::CollisionInfo info, int which);

    bool extraCollisionInformation(std::vector<double> configuration,
                                   std::vector<ompl::base::CollisionInfo>& collisionStructs);
                                   
    bool extraCollisionInformation(std::vector<double> configuration0, std::vector<double> configuration1,
                                    std::vector<ompl::base::ContinuousCollisionInfo>& collisionStructs);
private:
    trajopt::Configuration* rad_;
    trajopt::CollisionCheckerPtr coll_check_;
    std::map<const std::string, int> link2index_;
    std::vector<OpenRAVE::KinBody::LinkPtr> links_;
};

class OMPLOptSimplifier: public AOMPLPlanner {
public:
    OMPLOptSimplifier(OpenRAVE::EnvironmentBasePtr penv,
                      PlannerFactory const &simplifier_factory);
    virtual ~OMPLOptSimplifier();

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
                          PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

    virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters () const {
        return m_parameters;
    }

    bool GetCost(std::ostream & sout, std::istream & sin) const;

private:
    std::shared_ptr<TrajOptWrapper> m_wrapper;

    bool GetParametersCommand(std::ostream &sout, std::istream &sin) const;
};

typedef boost::shared_ptr<OMPLOptSimplifier> OMPLOptSimplifierPtr;

} // namespace or_ompl

#endif // OR_OMPL_OMPLOPTSIMPLIFIER_H_
