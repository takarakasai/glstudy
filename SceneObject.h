
#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H

#include <cstdio>
#include <vector>
#include <memory>

#include <eigen3/Eigen/Core>

//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
#include <GL/gl.h>

#include "dp_type.h"
#include "VertexObject.h"

#include "InterfaceSceneObject.h"

class SceneObject : public vaos, public/*implement*/ InterfaceSceneObject {
private:
  GLint gl_tmat_loc_id_;
  
  /* TODO integ constructor's rot/pos */
  Eigen::Matrix3d rot_;
  Eigen::Vector3d pos_;
  Dp::Math::real scale_;

public:
  SceneObject(size_t nov) : vaos(nov), rot_(Eigen::Matrix3d::Identity()), pos_(Eigen::Vector3d::Zero()), scale_(1.0) {
    gl_tmat_loc_id_ = 0;
  };

  virtual ~SceneObject() {};

  errno_t SetTransformMatrixLocId (GLint id) {
    if (gl_tmat_loc_id_ == -1) {
      return EINVAL;
    }

    gl_tmat_loc_id_ = id;

    return 0;
  }

  errno_t SetOffset(Eigen::Vector3d& pos, Eigen::Matrix3d& rot) {
    rot_ = rot;
    pos_ = pos;
  }

  errno_t SetScale(Dp::Math::real scale) {
    scale_ = scale;
  }

  errno_t Draw(Eigen::Matrix3d& Rot, Eigen::Vector3d& Pos) {
    if (gl_tmat_loc_id_ == -1) {
      fprintf(stderr, "ERROR %s (%d)\n", __PRETTY_FUNCTION__, gl_tmat_loc_id_);
      return -1;
    }

    /* TODO: why rot_ * Rot is NG? */
    Eigen::Matrix3d rot = Rot * (rot_ * scale_);
    //Eigen::Matrix3d rot = rot_ * Rot;
    Eigen::Vector3d pos = Pos + Rot * pos_;

    //GLfloat transformMatrix[16] = {
    //  (float)rot(0,0), (float)rot(0,1), (float)rot(0,2), 0.0,
    //  (float)rot(1,0), (float)rot(1,1), (float)rot(1,2), 0.0,
    //  (float)rot(2,0), (float)rot(2,1), (float)rot(2,2), 0.0,
    //  (float)pos(0),   (float)pos(1)  , (float)pos(2),   1.0
    //};
    GLfloat transformMatrix[16] = {
      (float)rot(0,0), (float)rot(1,0), (float)rot(2,0), 0.0,
      (float)rot(0,1), (float)rot(1,1), (float)rot(2,1), 0.0,
      (float)rot(0,2), (float)rot(1,2), (float)rot(2,2), 0.0,
      (float)pos(0),   (float)pos(1)  , (float)pos(2),   1.0
    };
 
    //  1.0, 0.00, 0.00, 0.0,
    //  0.0, 0.71,-0.71, 0.0,
    //  0.0, 0.71, 0.71, 0.0,
    //  0.0, 0.00, 1.00, 1.0
    //};



//    for (size_t i = 0; i < 4; i++) {
//      printf("%+7.2f %+7.2f %+7.2f %+7.2f\n", transformMatrix[i*4], transformMatrix[i*4+1], transformMatrix[i*4+2], transformMatrix[i*4+3]);
//    }

    //  1.0, 0.00, 0.00, 0.0,
    //  0.0, 0.71,-0.71, 0.0,
    //  0.0, 0.71, 0.71, 0.0,
    //  0.0, 0.00, 0.00, 1.0
    //};

    glUniformMatrix4fv(gl_tmat_loc_id_, 1, GL_FALSE, transformMatrix);

    ECALL(vaos::Draw());
  }
};

#endif

