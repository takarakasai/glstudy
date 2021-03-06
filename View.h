
#ifndef VIEW_H
#define VIEW_H

#include <cmath>

#include <eigen3/Eigen/Core>

/* TODO for GLfloat */
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "dp_type.h"

namespace ssg {

  void cameraMatrix(float fovy, float aspect, float near, float far, GLfloat *matrix);

  void lookAt(float ex, float ey, float ez,
              float tx, float ty, float tz,
              float ux, float uy, float uz,
              GLfloat *matrix);

  Eigen::Matrix4d cameraPerspectiveMatrix(Dp::Math::real fovy, Dp::Math::real aspect,
                                          Dp::Math::real near, Dp::Math::real far);

  Eigen::Matrix4d cameraParallelMatrix(Dp::Math::real width, Dp::Math::real aspect,
                                       Dp::Math::real near, Dp::Math::real far);

  Eigen::Matrix4d lookAt(
          Eigen::Vector3d camera_pos,
          Eigen::Vector3d camera_dir,
          Eigen::Vector3d camera_top = Eigen::Vector3d::UnitZ());
 
  Eigen::Matrix4d lookAt(
          Dp::Math::real ex, Dp::Math::real ey, Dp::Math::real ez,
          Dp::Math::real tx, Dp::Math::real ty, Dp::Math::real tz,
          Dp::Math::real ux, Dp::Math::real uy, Dp::Math::real uz);
}

#endif

