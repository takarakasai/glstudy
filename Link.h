
#include <list>
#include <string>

#include <memory>

#include "dp_type.h"

#include "CasCoords.h"

#include "Joint.h"

//#include <eigen3/Eigen/Geometry>

//using namespace std;
//using namespace Eigen;
//using namespace Dp::Math;

class Link : public CasCoords {
private:
  //const char* name_;
  std::string name_;

  std::shared_ptr<Joint> joint_;
  std::list<std::shared_ptr<Link>> clinks_;

  Eigen::Vector3d l_tippos_;
  Eigen::Matrix3d l_tiprot_;
  Dp::Math::real     mass_;
  Eigen::Vector3d centroid_; /* center of gravity */
  Eigen::Matrix3d cinertia_; /* inertia tensor at centroid */

  std::shared_ptr<Joint>& getJoint() {
    return joint_;
  }

public:
  //Link(const char* name) : name_(name) {};
  //Link(const char* name, std::shared_ptr<Joint> joint) : name_(name), joint_(joint) {};
  Link(
    const char* name, std::shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) :
    name_(name), joint_(joint), l_tippos_(lpos), l_tiprot_(Matrix3d::Identity()), mass_(mass), centroid_(centroid),
    cinertia_(cinertia) {};
    //cinertia_(Matrix3d::Identity()) {};
  virtual ~Link() {};

  void SetMass(Dp::Math::real mass) {
    mass_ = mass;
  }

  void SetCentroid(Vector3d centroid) {
    centroid_ = centroid;
  }

  void SetInertia(Matrix3d inertia) {
    cinertia_ = inertia;
  }

  static std::shared_ptr<Link> Create (
    const char* name, std::shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) {
    return std::make_shared<Link>(name, joint, lpos, mass, centroid, cinertia);
  }

  //std::shared_ptr<Joint> GetJoint() {
  //  return joint_;
  //}
  Joint& GetJoint() {
    return *(joint_);
  }

  std::shared_ptr<Joint> FindJoint(std::string& str) {
    /* TODO: duplicate */
    if (joint_->GetName() == str) {
      return joint_;
    }
    for (auto &link : clinks_) {
      auto joint = link->FindJoint(str);
      if (joint != NULL) {
        return joint;
      }
    }
    return NULL;
  }
  std::shared_ptr<Joint> FindJoint(const char* name) {
    std::string tname = name;
    return FindJoint(tname);
  }

  // TODO: --> CasCoords 
  Eigen::Vector3d& LTipPos() {
    return l_tippos_;
  }
  Eigen::Matrix3d& LTipRot() {
    return l_tiprot_;
  }

  errno_t AssignJoint (std::shared_ptr<Joint> joint) {
    joint_ = std::move(joint);
  }

  errno_t AddChild (std::shared_ptr<Link> clink) {
    CasCoords::AddChild(clink);
    clinks_.push_back(clink);
  }

  errno_t ApplyLocalCoords() {
    //CasCoords::LRot() = joint_->Rot() * l_tiprot_;
    CasCoords::LRot() = l_tiprot_ * joint_->Rot();
    CasCoords::LPos() = joint_->Pos() + l_tippos_;
    //std::cout << name_ << std::endl;
    //std::cout << "  ROT:" << CasCoords::LRot() << std::endl;
    //std::cout << "  POS:" << l_tippos_ << std::endl;
    //std::cout << "  POS:" << CasCoords::LPos() << std::endl;
    return 0;
  }

  errno_t UpdateLocals() {
    ECALL(ApplyLocalCoords());
    for (auto &link : clinks_) {
      link->UpdateLocals();
    }
    return 0;
  }

  errno_t UpdateCasCoords() {
    ECALL(UpdateLocals());      /* ローカル位置・姿勢更新 */
    ECALL(CasCoords::Update()); /* ワールド位置・姿勢更新 */
    return 0;
  }

  std::string& GetName() {
    return name_;
  }
};

