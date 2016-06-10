
#ifndef INTERFACE_SCENE_OBJECT_H
#define INTERFACE_SCENE_OBJECT_H

#include <eigen3/Eigen/Core>

/* TODO:remove */
#include <GL/gl.h>

#include "dp_type.h"

// interface
class InterfaceSceneObject {
private:
public:
  virtual errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) = 0;
  virtual errno_t SetTransformMatrixLocId (GLint id) = 0;
  //errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) {return 0;};
  //errno_t SetTransformMatrixLocId (GLint id) {return 0;};

  /* TODO */
  virtual errno_t SetOffset(Eigen::Vector3d& pos, Eigen::Matrix3d& rot) = 0;
  virtual errno_t SetScale(Dp::Math::real scale) = 0;

  //InterfaceSceneObject() {};
  virtual ~InterfaceSceneObject() {};
};

#endif

