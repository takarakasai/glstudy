
#ifndef VIEW_H
#define VIEW_H

#include <cmath>

/* TODO for GLfloat */
#include <GL/gl.h>

#include "dp_type.h"

namespace ssg {

  void cameraMatrix(float fovy, float aspect, float near, float far, GLfloat *matrix);
 
  void lookAt(float ex, float ey, float ez,
              float tx, float ty, float tz,
              float ux, float uy, float uz,
              GLfloat *matrix);

}

#endif

