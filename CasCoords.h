
#ifndef CASCOORDS_H
#define CASCOORDS_H

#include <list>
#include <memory>
#include <eigen3/Eigen/Core>

#include <iostream>

using Eigen::Vector3d;
using Eigen::Matrix3d;

#define ECALL(function)     \
  do {                      \
    errno_t eno = function; \
    if (eno != 0) {         \
      return eno;           \
    }                       \
  } while(0)

class Coordinates {
private:
  Vector3d pos;
  Matrix3d rot;

public:
  Coordinates (
    Vector3d position = Vector3d(0.0,0.0,0.0))
    : pos(position) {
    rot << Matrix3d::Identity();
  };

  Vector3d& Pos() {
    return pos;
  }

  Matrix3d& Rot() {
    return rot;
  }

  Coordinates& operator*= (Coordinates &parent) {
    // child * parent;
    pos = parent.Pos() + parent.Rot() * pos;
    rot =                parent.Rot() * rot;
    return (*this);
  }

  Coordinates operator* (Coordinates &parent) {
    // child * parent;
    Coordinates ans;
    ans.Pos() = parent.Pos() + parent.Rot() * pos;
    ans.Rot() =                parent.Rot() * rot;
    //printf("%p %lf %lf %lf\n", this, ans.Pos()(0), ans.Pos()(1), ans.Pos()(2));
    return ans;
  }

  Coordinates operator/= (Coordinates &basis) {
    // position/rotation from basis;
    pos = pos - basis.Pos();
    rot = rot * basis.Rot().transpose();
    return (*this);
  }

  Coordinates operator/ (Coordinates &basis) {
    // position/rotation from basis;
    Coordinates ans;
    ans.Pos() = pos - basis.Pos();
    ans.Rot() = rot * basis.Rot().transpose();
    return ans;
  }
};

#if 0
class Tree : public std::enable_shared_from_this<Tree> {
private:
  std::weak_ptr<Tree> pnode;
  std::list<std::weak_ptr<Tree>> cnodes;
private:
  void SetParent (std::weak_ptr<Tree> node) {
    pnode = node;
  }

public:
  Tree() {};
  virtual ~Tree() {};

  std::weak_ptr<Tree>& GetParent() {
    return pnode;
  }

  std::list<std::weak_ptr<Tree>>& GetChilds () {
    return cnodes;
  }

  void AddChild (std::weak_ptr<Tree> node) {
    node.lock()->SetParent(this->shared_from_this());
    cnodes.push_back(node);
  }
};
#endif

class CasCoords : public std::enable_shared_from_this<CasCoords> {
private:
  Coordinates lcoords; /* local coordinates */
  Coordinates wcoords; /* world coordinates */

  std::weak_ptr<CasCoords> pnode;
  std::list<std::weak_ptr<CasCoords>> cnodes;

private:
  void SetParent (std::weak_ptr<CasCoords> node) {
    pnode = node;
  }

public:

  CasCoords() {
  }

  virtual ~CasCoords() {
  }

  std::weak_ptr<CasCoords>& GetParent() {
    return pnode;
  }

  std::list<std::weak_ptr<CasCoords>>& GetChilds () {
    return cnodes;
  }

  void AddChild (std::weak_ptr<CasCoords> node) {
    node.lock()->SetParent(this->shared_from_this());
    cnodes.push_back(node);
  }

  Vector3d& LPos() {
    return lcoords.Pos();
  }
  Matrix3d& LRot() {
    return lcoords.Rot();
  }

  Vector3d& WPos() {
    return wcoords.Pos();
  }
  Matrix3d& WRot() {
    return wcoords.Rot();
  }

  Coordinates& Local() {
    return lcoords;
  }

  Coordinates& World() {
    return wcoords;
  }

  CasCoords operator*= (CasCoords &parent) {
    wcoords = lcoords * parent.World();
    return (*this);
  }

  CasCoords operator* (CasCoords &parent) {
    CasCoords ans;
    ans.World() = lcoords * parent.World();
    ans.Local() = lcoords;
    return ans;
  }

private:
  errno_t update (CasCoords* parent) {
    if (parent != NULL) {
      (*this) *= (*parent);
      //std::cout << "------------" << std::endl;
      //std::cout << "  ROT:" << WRot() << std::endl;
      //std::cout << "  POS:" << WPos() << std::endl;
    }

    for (auto& node : cnodes) {
      node.lock()->update(this);
    }

    return 0;
  }

public:
  virtual errno_t Exec (void) { return -1;}

  errno_t ExecAll (void) {
    Exec();
    for (auto& node : cnodes) {
      node.lock()->ExecAll();
    }

    return 0;
  };

  errno_t Update (void) {
    ECALL(update(NULL));
    return 0;
  };
};

#endif
