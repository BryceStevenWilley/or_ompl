#include "collision_checker.hpp"
#include "rave_utils.hpp"
using namespace OpenRAVE;

namespace trajopt {

boost::shared_ptr<CollisionChecker> CollisionChecker::GetOrCreate(OR::EnvironmentBase& env) {
  UserDataPtr ud = GetUserData(env, "trajopt_cc");
  if (!ud) {
    RAVELOG_INFO("creating bullet collision checker for environment");
    ud =  CreateCollisionChecker(env.shared_from_this());
    SetUserData(env, "trajopt_cc", ud);
  }
  else {
    RAVELOG_DEBUG("already have a collision checker for this environment");
  }
  return boost::dynamic_pointer_cast<CollisionChecker>(ud);
}

void CollisionChecker::IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body) {
  RAVELOG_DEBUG("IgnoreZeroStateSelfCollisions for %s", body->GetName().c_str());
  OR::KinBody::KinBodyStateSaver saver(body);
  body->SetDOFValues(DblVec(body->GetDOF(), 0));
  body->SetTransform(Transform(Vector(1,0,0,0), (Vector(0,0,10))));

  std::vector<Collision> collisions;
  BodyVsAll(*body,  collisions);
  RAVELOG_DEBUG("%li extra self collisions in zero state", collisions.size());
  for(int i=0; i < collisions.size(); ++i) {
    RAVELOG_DEBUG("ignoring self-collision: %s %s", collisions[i].linkA->GetName().c_str(), collisions[i].linkB->GetName().c_str());
    ExcludeCollisionPair(*collisions[i].linkA, *collisions[i].linkB);
  }
  RAVELOG_DEBUG("------");
}

void CollisionChecker::IgnoreZeroStateSelfCollisions() {
  std::vector<OR::KinBodyPtr> bodies;
  GetEnv()->GetBodies(bodies);

  for (const OR::KinBodyPtr& body : bodies) {
    IgnoreZeroStateSelfCollisions(body);
  }
}

std::ostream& operator<<(std::ostream& o, const Collision& c) {
  o << (c.linkA ? c.linkA->GetName() : "NULL") << "--" <<  (c.linkB ? c.linkB->GetName() : "NULL") <<
      " distance: " << c.distance <<
      " normal: " << c.normalB2A <<
      " ptA: " << c.ptA <<
      " ptB: " << c.ptB <<
      " time: " << c.time <<
      " weight: " << c.weight;
  return o;
}

} // namespace trajopt
