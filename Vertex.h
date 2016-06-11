
#ifndef VERTEX_H
#define VERTEX_H

#include <vector>
#include <eigen3/Eigen/Core>

/* TODO: */
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "dp_type.h"

//using namespace Eigen;

namespace ssg {

  typedef struct sVertices {
    std::vector<Eigen::Vector3f> poses; /* position vectors */
    std::vector<Eigen::Vector3f> norms; /* normal vectors   */
  
    void push (Eigen::Vector3f pos) {
      poses.push_back(pos);
      norms.push_back(pos / pos.norm());
    }
  
    void push (Eigen::Vector3f pos, Eigen::Vector3f norm) {
      poses.push_back(pos);
      norms.push_back(norm);
    }
  
    size_t size () {
      return poses.size();
    }
  
    void rotate (Eigen::Matrix3f rot) {
      for (auto &pose: poses) {
        pose = rot * pose;
      }
      for (auto &norm: norms) {
        norm = rot * norm;
      }
    }
  
  } Vertices;
  
  Vertices cylinderVertices (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors);
  
  Vertices sphereVertices (GLfloat radius, size_t nor, size_t noh);
 
  Vertices coneVertices (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors);
 
  Vertices rectangularVertices (Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height);

}

#endif

