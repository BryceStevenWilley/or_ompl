#include <Eigen/Core>
#include <or_ompl/TrajOpt/rave_utils.hpp>

using namespace OpenRAVE;
using namespace std;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

namespace trajopt {

OR::KinBody::LinkPtr GetLinkMaybeAttached(RobotBasePtr robot, const string& name) {
  OR::KinBody::LinkPtr link = robot->GetLink(name);
  if (link) return link;
  std::set<OR::KinBodyPtr> setAttached;
  robot->GetAttached(setAttached);
  for (const OR::KinBodyPtr& body : setAttached) {
    OR::KinBody::LinkPtr link = body->GetLink(name);
    if (link) return link;
  }
  return link;
}

RobotBase::ManipulatorPtr GetManipulatorByName(RobotBase& robot, const std::string& name) {
  vector<RobotBase::ManipulatorPtr> manips = robot.GetManipulators();
  for (RobotBase::ManipulatorPtr& manip : manips) {
    if (manip->GetName()==name) return manip;
  }
  return RobotBase::ManipulatorPtr();
}

OR::KinBodyPtr GetBodyByName(EnvironmentBase& env, const std::string& name) {
  vector<OR::KinBodyPtr> bodies;
  env.GetBodies(bodies);
  for (OR::KinBodyPtr& body : bodies) {
    if (body->GetName() == name) return body;
  }
  return OR::KinBodyPtr();
}

RobotBasePtr GetRobotByName(EnvironmentBase& env, const std::string& name) {
  return boost::dynamic_pointer_cast<RobotBase>(GetBodyByName(env, name));
}

RobotBasePtr GetRobot(EnvironmentBase& env) {
  vector<RobotBasePtr> robots;
  env.GetRobots(robots);
  if (robots.size() == 0) {
    RAVELOG_ERROR("no robots!");
    return RobotBasePtr();
  }
  else if (robots.size() == 1) {
    return robots[0];
  }
  else {
    RAVELOG_ERROR("I don't know which robot you want");
    return RobotBasePtr();
  }
}

int GetRobotLinkIndex(const RobotBase& robot, const OR::KinBody::Link& link) {
  if (link.GetParent().get() == &robot) return link.GetIndex();
  else if (OR::KinBody::LinkPtr grabber = robot.IsGrabbing(link.GetParent())) {
    return grabber->GetIndex();
  }
  else {
    throw OR::openrave_exception("link not part of robot");
  }
}

bool DoesAffect(const RobotBase& robot, const vector<int>& dof_inds, int link_ind) {
  for (const int& dof_ind : dof_inds) {
    if (robot.DoesAffect(dof_ind, link_ind)) return true;
  }
  return false;
}

void PlotAxes(EnvironmentBase& env, const OpenRAVE::Transform& T, float size, vector<GraphHandlePtr>& handles) {
  TransformMatrix mrave = OR::geometry::matrixFromQuat(T.rot);
  DblMatrix m = Eigen::Map<DblMatrix>(mrave.m,3,4);
  Vector x = T.trans + Vector(m(0,0), m(1,0), m(2,0))*size,
         y = T.trans + Vector(m(0,1), m(1,1), m(2,1))*size,
         z = T.trans + Vector(m(0,2), m(1,2), m(2,2))*size;
  handles.push_back(env.drawarrow(T.trans, x, size/10, Vector(1,0,0,1)));
  handles.push_back(env.drawarrow(T.trans, y, size/10, Vector(0,1,0,1)));
  handles.push_back(env.drawarrow(T.trans, z, size/10, Vector(0,0,1,1)));
}


}
