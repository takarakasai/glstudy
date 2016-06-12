
#include "View.h"

#include "dp_type.h"

#include <iostream>

namespace ssg {

  Eigen::Matrix4d cameraPerspectiveMatrix(Dp::Math::real fovy, Dp::Math::real aspect,
                                          Dp::Math::real near, Dp::Math::real far) {
      Dp::Math::real f  = 1.0 / tanf(Dp::Math::deg2rad(fovy * 0.5));
      Dp::Math::real dz = far - near;

      Eigen::Matrix4d cmat;
      cmat <<
          f / aspect, 0.0,              0.0,  0.0,
                 0.0,   f,              0.0,  0.0,
                 0.0, 0.0,   -(far+near)/dz, -1.0,
                 0.0, 0.0, -2.0*far*near/dz,  0.0;

      return cmat;
  }

  Eigen::Matrix4d cameraParallelMatrix(Dp::Math::real width, Dp::Math::real aspect,
                                       Dp::Math::real near, Dp::Math::real far) {
      Dp::Math::real dz = far - near;

      Eigen::Matrix4d cmat;
      cmat <<
           2.0 / width,                  0.0,              0.0,  0.0,
                   0.0, 2.0 / width * aspect,              0.0,  0.0,
                   0.0,                  0.0,          -2.0/dz,  0.0,
                   0.0,                  0.0,   -(far+near)/dz,  1.0;

      return cmat;
  }

  void cameraMatrix(float fovy, float aspect, float near, float far, GLfloat *matrix) {
    float f = 1.0f / tanf(fovy * 0.5f * 3.141593f / 180.0f);
    float dz = far - near;
    
    matrix[ 0] = f / aspect;
    matrix[ 5] = f;
    matrix[10] = -(far + near) / dz;
    matrix[11] = -1.0f;
    matrix[14] = -2.0f * far * near / dz;
    matrix[ 1] = matrix[ 2] = matrix[ 3] = matrix[ 4] =
    matrix[ 6] = matrix[ 7] = matrix[ 8] = matrix[ 9] =
    matrix[12] = matrix[13] = matrix[15] = 0.0f;
  }

  Eigen::Matrix4d lookAt(
          Eigen::Vector3d camera_pos,
          Eigen::Vector3d camera_dir,
          Eigen::Vector3d camera_top) {
          //Eigen::Vector3d camera_top = Eigen::Vector3f::UnitZ()) {
    Eigen::Matrix3d mat3;
    Eigen::Matrix4d mat;
    Eigen::Vector3d t;

    // Dp::Math::real ex, Dp::Math::real ey, Dp::Math::real ez,
    // Dp::Math::real tx, Dp::Math::real ty, Dp::Math::real tz,
    // Dp::Math::real ux, Dp::Math::real uy, Dp::Math::real uz) {
    //

    t = camera_pos - camera_dir;
    auto l = t.norm();
    mat3.col(2) = t / l;
    //tx = ex - tx;
    //ty = ey - ty;
    //tz = ez - tz;
    //l = sqrtf(tx * tx + ty * ty + tz * tz);
    //mat(0, 2) = tx / l;
    //mat(1, 2) = ty / l;
    //mat(2, 2) = tz / l;

    t = camera_top.cross(t);
    l = t.norm();
    mat3.col(0) = t / l;
    //tx = uy * mat(2, 2) - uz * mat(1, 2);
    //ty = uz * mat(0, 2) - ux * mat(2, 2);
    //tz = ux * mat(1, 2) - uy * mat(0, 2);
    //l = sqrtf(tx * tx + ty * ty + tz * tz);
    //mat(0, 0) = tx / l;
    //mat(1, 0) = ty / l;
    //mat(2, 0) = tz / l;

    mat3.col(1) = mat3.col(2).cross(mat3.col(0));
    //mat(0, 1) = mat(1, 2) * mat(2, 0) - mat(2, 2) * mat(1, 0);
    //mat(1, 1) = mat(2, 2) * mat(0, 0) - mat(0, 2) * mat(2, 0);
    //mat(2, 1) = mat(0, 2) * mat(1, 0) - mat(1, 2) * mat(0, 0);

    mat(3, 0) = -(camera_pos.dot(mat3.col(0)));
    mat(3, 1) = -(camera_pos.dot(mat3.col(1)));
    mat(3, 2) = -(camera_pos.dot(mat3.col(2)));
    //mat(3, 0) = -(ex * mat(0, 0) + ey * mat(1, 0) + ez * mat(2, 0));
    //mat(3, 1) = -(ex * mat(0, 1) + ey * mat(1, 1) + ez * mat(2, 1));
    //mat(3, 2) = -(ex * mat(0, 2) + ey * mat(1, 2) + ez * mat(2, 2));

    mat.block(0,0,3,3) = mat3;
    mat(0, 3) = mat(1, 3) = mat(2, 3) = 0.0;
    mat(3, 3) = 1.0;

    return mat;
  }

  Eigen::Matrix4d lookAt(
          Dp::Math::real ex, Dp::Math::real ey, Dp::Math::real ez,
          Dp::Math::real tx, Dp::Math::real ty, Dp::Math::real tz,
          Dp::Math::real ux, Dp::Math::real uy, Dp::Math::real uz) {
    Dp::Math::real l;
    Eigen::Matrix4d mat;

    tx = ex - tx;
    ty = ey - ty;
    tz = ez - tz;
    l = sqrtf(tx * tx + ty * ty + tz * tz);
    mat(0, 2) = tx / l;
    mat(1, 2) = ty / l;
    mat(2, 2) = tz / l;

    tx = uy * mat(2, 2) - uz * mat(1, 2);
    ty = uz * mat(0, 2) - ux * mat(2, 2);
    tz = ux * mat(1, 2) - uy * mat(0, 2);
    l = sqrtf(tx * tx + ty * ty + tz * tz);
    mat(0, 0) = tx / l;
    mat(1, 0) = ty / l;
    mat(2, 0) = tz / l;

    mat(0, 1) = mat(1, 2) * mat(2, 0) - mat(2, 2) * mat(1, 0);
    mat(1, 1) = mat(2, 2) * mat(0, 0) - mat(0, 2) * mat(2, 0);
    mat(2, 1) = mat(0, 2) * mat(1, 0) - mat(1, 2) * mat(0, 0);
    
    mat(3, 0) = -(ex * mat(0, 0) + ey * mat(1, 0) + ez * mat(2, 0));
    mat(3, 1) = -(ex * mat(0, 1) + ey * mat(1, 1) + ez * mat(2, 1));
    mat(3, 2) = -(ex * mat(0, 2) + ey * mat(1, 2) + ez * mat(2, 2));
  
    mat(0, 3) = mat(1, 3) = mat(2, 3) = 0.0;
    mat(3, 3) = 1.0;
  
    return mat;
  }

  void lookAt(float ex, float ey, float ez,
              float tx, float ty, float tz,
              float ux, float uy, float uz,
              GLfloat *matrix)
  {
    float l;
  
    /* z ¼´ = e - t */
    tx = ex - tx;
    ty = ey - ty;
    tz = ez - tz;
    l = sqrtf(tx * tx + ty * ty + tz * tz);
    matrix[ 2] = tx / l;
    matrix[ 6] = ty / l;
    matrix[10] = tz / l;
  
    /* x ¼´ = u x z ¼´ */
    tx = uy * matrix[10] - uz * matrix[ 6];
    ty = uz * matrix[ 2] - ux * matrix[10];
    tz = ux * matrix[ 6] - uy * matrix[ 2];
    l = sqrtf(tx * tx + ty * ty + tz * tz);
    matrix[ 0] = tx / l;
    matrix[ 4] = ty / l;
    matrix[ 8] = tz / l;
  
    /* y ¼´ = z ¼´ x x ¼´ */
    matrix[ 1] = matrix[ 6] * matrix[ 8] - matrix[10] * matrix[ 4];
    matrix[ 5] = matrix[10] * matrix[ 0] - matrix[ 2] * matrix[ 8];
    matrix[ 9] = matrix[ 2] * matrix[ 4] - matrix[ 6] * matrix[ 0];
  
    /* Ê¿¹Ô°ÜÆ° */
    matrix[12] = -(ex * matrix[ 0] + ey * matrix[ 4] + ez * matrix[ 8]);
    matrix[13] = -(ex * matrix[ 1] + ey * matrix[ 5] + ez * matrix[ 9]);
    matrix[14] = -(ex * matrix[ 2] + ey * matrix[ 6] + ez * matrix[10]);
  
    /* »Ä¤ê */
    matrix[ 3] = matrix[ 7] = matrix[11] = 0.0f;
    matrix[15] = 1.0f;
  }

}

