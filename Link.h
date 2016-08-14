
#ifndef LINK_H
#define LINK_H

#include <list>
#include <string>

#include <memory>

#include "dp_type.h"

#include "CasCoords.h"

#include "Joint.h"

// TODO: remove
#include "InterfaceSceneObject.h"

class InterfaceSceneObject;

class Link : public CasCoords {
private:
  //const char* name_;
  std::string name_;

  std::shared_ptr<Joint> joint_;

  Eigen::Vector3d l_tippos_;
  Eigen::Matrix3d l_tiprot_;
  Dp::Math::real     mass_;
  Eigen::Vector3d centroid_; /* center of gravity at local coordinates */
  Eigen::Vector3d wcentroid_; /* center of gravity at world coordinates */
  Eigen::Matrix3d cinertia_; /* inertia tensor at centroid at local coordinates */

  std::shared_ptr<Joint>& getJoint() {
    return joint_;
  }

protected:
  /* TODO */
  //std::shared_ptr<Link> parent_ = NULL;
  Link *parent_ = NULL;
  std::list<std::shared_ptr<Link>> clinks_;

public:
  //Link(const char* name) : name_(name) {};
  //Link(const char* name, std::shared_ptr<Joint> joint) : name_(name), joint_(joint) {};
  Link(
    const char* name, std::shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) :
    name_(name), joint_(joint), l_tippos_(lpos), l_tiprot_(Matrix3d::Identity()), mass_(mass), centroid_(centroid),
    wcentroid_(Eigen::Vector3d::Zero()), cinertia_(cinertia) {};
    //cinertia_(Matrix3d::Identity()) {};
  virtual ~Link() {};

  void SetMass(Dp::Math::real mass) {
    mass_ = mass;
  }
  Dp::Math::real& GetMass() {
    return mass_;
  }

  void SetCentroid(Vector3d centroid) {
    centroid_ = centroid;
  }
  Vector3d& GetCentroid() {
    return centroid_;
  }
  Vector3d& GetWCentroid() {
    /* TODO: update at UpdateCasCoords */
    wcentroid_ = WPos() + WRot() * centroid_;
    return wcentroid_;
  }

  void SetInertia(Matrix3d inertia) {
    cinertia_ = inertia;
  }
  Matrix3d& GetIntertia() {
    return cinertia_;
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

  std::shared_ptr<Joint> FindJoint(const std::string& str) {
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

  /* TODO: not Link* */
  //std::shared_ptr<Link> FindLink(std::string& str) {
  Link* FindLink(const std::string& str) {
    /* TODO: duplicate */
    if (GetName() == str) {
      return this;
    }
    for (auto &link : clinks_) {
      auto lnk = link->FindLink(str);
      if (lnk != NULL) {
        return lnk;
      }
    }
    return NULL;
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
    return 0;
  }

  //std::shared_ptr<Link> GetParent () {
  Link* GetParent () {
    return parent_;
  }

  errno_t SetParent (Link *link) {
    parent_ = link;
    return 0;
  }

  std::list<std::shared_ptr<Link>> GetChilds () {
    return clinks_;
  }

  errno_t AddChild (std::shared_ptr<Link> clink) {
    CasCoords::AddChild(clink);

    clink->SetParent(this);
    clinks_.push_back(clink);

    return 0;
  }

  errno_t ApplyLocalCoords() {
    /* pos & lot root of this link */
    //CasCoords::LRot() = joint_->Rot() * l_tiprot_;
    //std::cout << name_ << std::endl;
    CasCoords::LRot() = l_tiprot_ * joint_->Rot();
    CasCoords::LPos() = joint_->Pos() + l_tippos_;
    //std::cout << name_ << std::endl;
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


  /* Coexistance of DrawableLink & Link */
  /* TODO: int32_t --> GLint */
  virtual errno_t SetDrawMode (InterfaceSceneObject::DrawMode mode) {
    return 0;
  }
  virtual errno_t SetTransformMatrixLocId (int32_t id) {
    return 0;
  }
  virtual errno_t SetTextureLocId (int32_t id) {
    return 0;
  }
  virtual errno_t SetMaterialColorLocId (int32_t id) {
    return 0;
  }
  virtual void AddShape (std::shared_ptr<InterfaceSceneObject> obj) {
    std::cout << "  ERROR-------------------------------\n";
    return;
  }
  virtual void AddShape (std::list<std::shared_ptr<InterfaceSceneObject>> objs) {
    std::cout << "  ERROR-------------------------------\n";
    return;
  }
};

#endif

