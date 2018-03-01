#pragma once
#include <openrave/openrave.h>
#include <Eigen/Core>

namespace trajopt {

namespace OR = OpenRAVE;
typedef std::vector<double> DblVec;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

class Configuration {
public:
  
  virtual void SetDOFValues(const DblVec& dofs) = 0;
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
  virtual DblVec GetDOFValues() = 0;
  virtual int GetDOF() const = 0;
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() = 0;
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const = 0;
  virtual DblMatrix RotationJacobian(int link_ind) const = 0;
  virtual bool DoesAffect(const OR::KinBody::Link& link) = 0;
  virtual std::vector<OR::KinBodyPtr> GetBodies() = 0;
  virtual std::vector<OR::KinBody::LinkPtr> GetAffectedLinks() = 0;
  virtual void GetAffectedLinks(std::vector<OR::KinBody::LinkPtr>& links, bool only_with_geom, std::vector<int>& link_inds) = 0;
  virtual DblVec RandomDOFValues() = 0;  
  virtual ~Configuration() {}
  struct Saver {
    virtual ~Saver(){}
  };
  typedef boost::shared_ptr<Saver> SaverPtr;
  struct GenericSaver : public Saver {
    DblVec dofvals;
    Configuration* parent;
    GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
    ~GenericSaver() {
      parent->SetDOFValues(dofvals);
    }
  }; // inefficient
  
  virtual SaverPtr Save() {
    return SaverPtr(new GenericSaver(this));
  }


};
typedef boost::shared_ptr<Configuration> ConfigurationPtr;

/**
Stores an OpenRAVE robot and the active degrees of freedom  
*/
class RobotAndDOF : public Configuration {
public:
  RobotAndDOF(OR::KinBodyPtr _robot, const std::vector<int>& _joint_inds, int _affinedofs=0, const OR::Vector _rotationaxis=OR::Vector(0,0,1)) :
    robot(_robot), joint_inds(_joint_inds), affinedofs(_affinedofs), rotationaxis(_rotationaxis) {}

  void SetDOFValues(const DblVec& dofs);
  void GetDOFLimits(DblVec& lower, DblVec& upper) const;
  DblVec GetDOFValues();
  int GetDOF() const;
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {return robot->GetEnv();};  
  std::vector<int> GetJointIndices() const {return joint_inds;}
  DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const;
  DblMatrix RotationJacobian(int link_ind) const;
  OR::RobotBasePtr GetRobot() const {return boost::dynamic_pointer_cast<OR::RobotBase>(robot);}
  virtual std::vector<OpenRAVE::KinBodyPtr> GetBodies();  
  bool DoesAffect(const OR::KinBody::Link& link);
  std::vector<OR::KinBody::LinkPtr> GetAffectedLinks();
  void GetAffectedLinks(std::vector<OR::KinBody::LinkPtr>& links, bool only_with_geom, std::vector<int>& link_inds);
  DblVec RandomDOFValues();

  struct RobotSaver : public Saver {
    OR::KinBody::KinBodyStateSaver saver;
    RobotSaver(OR::KinBodyPtr robot) : saver(robot) {}
  };
  SaverPtr Save() {
    return SaverPtr(new RobotSaver(robot));
  }
  void SetRobotActiveDOFs();
  
private:
  OR::KinBodyPtr robot;
  std::vector<int> joint_inds;
  int affinedofs;
  OR::Vector rotationaxis;
};
typedef boost::shared_ptr<RobotAndDOF> RobotAndDOFPtr;

}
