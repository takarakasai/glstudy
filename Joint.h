
#ifndef JOINT_H
#define JOINT_H

#include <list>
#include <string>

#include "dp_type.h"

#include "Actuator.h"

//#include <eigen3/Eigen/Geometry>

namespace Eigen {
  typedef Matrix< double, 6, 1 >  Vector6d;
  typedef Matrix< double, 6, 6 >  Matrix6d;
}

#include <iostream>

//class Joint : public CasCoords {
class Joint {
private:
  //const char* name_;
  std::string name_;

protected:
  std::list<std::shared_ptr<Actuator>> actuators_;

  /* configuration */
  Eigen::Vector6d inertia_   = Eigen::Vector6d::Zero();
  Eigen::Vector6d viscosity_ = Eigen::Vector6d::Zero();
  Eigen::Vector6d min_val_   = Eigen::Vector6d::Zero();
  Eigen::Vector6d max_val_   = Eigen::Vector6d::Zero();

  Eigen::Vector6d oval_      = Eigen::Vector6d::Zero(); /* offset value */
  Eigen::Vector6d val_       = Eigen::Vector6d::Zero();
  Eigen::Vector6d vel_       = Eigen::Vector6d::Zero();
  Eigen::Vector6d acc_       = Eigen::Vector6d::Zero();
  Eigen::Vector6d efrc_      = Eigen::Vector6d::Zero(); /* external torque */

  Eigen::Vector6d afrc_      = Eigen::Vector6d::Zero(); /* actuation torque */

  /* temporary rot/pos */
  Eigen::Matrix3d rot_       = Eigen::Matrix3d::Identity();
  Eigen::Vector3d pos_       = Eigen::Vector3d::Zero();

public:
  Joint(const char* name) : name_(name) {
    //rot_ = Eigen::Matrix3d::Identity();
    //pos_ = Eigen::Vector3d::Zero();
  };
  virtual ~Joint() {};

  void SetInertia(size_t idx, Dp::Math::real val) {
    inertia_(idx) = val;
  }

  void SetViscosity(size_t idx, Dp::Math::real val) {
    viscosity_(idx) = val;
  }

  void SetRange(size_t idx, Dp::Math::real min, Dp::Math::real max) {
    min_val_(idx) = min;
    max_val_(idx) = max;
  }

  //virtual errno_t ApplyLocalCoords() = 0;

  size_t GetNumOfActuators() {
    return actuators_.size();
  }

  std::list<std::shared_ptr<Actuator>>& Actuators() {
    return actuators_;
  }

  virtual Eigen::Matrix3d& Rot() {
    return rot_;
  }

  virtual Eigen::Vector3d& Pos() {
    return pos_;
  }

  /* TODO: remove */
  virtual Eigen::Vector3d Axis() {
    return Eigen::Vector3d::Zero();
  }

  const Eigen::Vector6d& GetOffsetValue() { return oval_;};
  const Eigen::Vector6d& GetValue() { return val_;};
  const Eigen::Vector6d& GetVeloc() { return vel_;};
  const Eigen::Vector6d& GetAccel() { return acc_;};
  const Eigen::Vector6d& GetExtFrc() { return efrc_;};
  const Eigen::Vector6d& GetActFrc() { return afrc_;};

  void SetOffsetValue (Eigen::Vector6d& ovalue) {oval_ = ovalue;};
  void SetValue (Eigen::Vector6d& value) {val_ = value;};
  void SetVeloc (Eigen::Vector6d& veloc) {vel_ = veloc;};
  void SetAccel (Eigen::Vector6d& accel) {acc_ = accel;};
  void SetExtFrc (Eigen::Vector6d& extfrc) {efrc_ = extfrc;};
  void SetActFrc (Eigen::Vector6d& actfrc) {afrc_ = actfrc;};

  // TODO:
  virtual errno_t SetOffsetValue (Dp::Math::real ovalue) {oval_(0) = ovalue; return 0;};
  virtual errno_t SetValue (Dp::Math::real value) {val_(0) = value; return 0;};
  virtual errno_t SetAngle (Dp::Math::real value) {val_(0) = value; return 0;};
  virtual Dp::Math::real GetOffsetAngle () {return oval_(0);};
  virtual Dp::Math::real GetAngle () {return val_(0);};

  std::string& GetName() { return name_;};
};

class RotaryJoint : public Joint {
private:
  Eigen::Vector3d axis_;
  Dp::Math::real& oangle_; /* offset angle */
  Dp::Math::real& angle_;

public:
  RotaryJoint(const char* name, Eigen::Vector3d axis) :
    Joint(name),
    axis_(axis), 
    oangle_(Joint::oval_(0)),
    angle_(Joint::val_(0))
  {
    actuators_.push_back(
      std::make_shared<Actuator>(oval_(0), val_(0), vel_(0), acc_(0), efrc_(0), afrc_(0))
    );
  };

  virtual ~RotaryJoint() {};

  static std::shared_ptr<RotaryJoint> Create (const char* name, Eigen::Vector3d axis) {
    return std::make_shared<RotaryJoint>(name, axis);
  }

  Eigen::Matrix3d& Rot() {
    rot_ = Eigen::AngleAxisd(oangle_ + angle_, axis_);
    return rot_;
  }

  /* TODO: remove */
  /* TODO: for simulator */
  Eigen::Vector3d Axis() {
    return axis_;
  }

  //errno_t ApplyLocalCoords() {
  //  CasCoords::LRot() = rot();
  //  return 0;
  //}

  errno_t SetAngle (Dp::Math::real angle) {
    angle_ = angle;
    return 0; 
  }

  Dp::Math::real GetOffsetAngle () {
    return oangle_;
  }
  Dp::Math::real GetAngle () {
    return angle_;
  }
};

#endif

