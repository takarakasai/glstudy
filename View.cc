
#include "View.h"

namespace ssg {

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

