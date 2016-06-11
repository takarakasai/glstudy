
#include "Vertex.h"

#include "Utils.h"

//using namespace Eigen;

namespace ssg {

  Vertices cylinderVertices (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors) {
    Vertices verts;
  
    const GLfloat r = radius;
    auto circle  = ssg::circle_tbl(sectors);
  
    verts.push((Eigen::Vector3f(0, 0, +height/2.0) + pos));
    verts.push((Eigen::Vector3f(0, 0, -height/2.0) + pos));
  
    for (auto vec : circle) {
      auto rvec = r * vec;
      Eigen::Vector3f pvec(rvec(0), rvec(1), +height/2.0);
      verts.push(pvec + pos);
    }
  
    for (auto vec : circle) {
      auto rvec = r * vec;
      Eigen::Vector3f pvec(rvec(0), rvec(1), -height/2.0);
      verts.push(pvec + pos);
    }
  
    return verts;
  }
  
  Vertices sphereVertices (Eigen::Vector3f pos, GLfloat radius, size_t nor, size_t noh) {
    Vertices verts;
  
    const GLfloat r = radius;
    auto circle  = ssg::circle_tbl(nor);
  
    verts.push(Eigen::Vector3f(0, 0, +r) + pos);
  
    for (size_t i = 1; i < noh; i++) {
      auto rad = Dp::Math::PI * i / noh;
      auto h   = r * cos(rad);
      auto rv  = r * sin(rad);
  
      for (auto vec : circle) {
        auto rvec = rv * vec;
        verts.push(Eigen::Vector3f(rvec(0), rvec(1), h) + pos);
      }
    }
  
    verts.push(Eigen::Vector3f(0, 0, -r) + pos);
  
    return verts;
  }
  
  Vertices coneVertices (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors) {
    Vertices verts;
  
    const GLfloat r = radius;
    auto circle  = ssg::circle_tbl(sectors);
  
    verts.push(Eigen::Vector3f(0, 0, +height) + pos);
  
    for (auto vec : circle) {
      auto rvec = r * vec;
      Eigen::Vector3f pvec(rvec(0), rvec(1), 0.0);
      verts.push(pvec + pos);
    }
  
    verts.push(Eigen::Vector3f(0, 0, 0) + pos);
  
    return verts;
  }
  
  Vertices rectangularVertices (Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height) {
    Vertices verts;
  
    auto lx = width  / 2.0;
    auto ly = length / 2.0;
    auto lz = height / 2.0;
  
    verts.push(Eigen::Vector3f(-lx, -ly, -lz) + pos);
    verts.push(Eigen::Vector3f(-lx, -ly, +lz) + pos);
    verts.push(Eigen::Vector3f(-lx, +ly, -lz) + pos);
    verts.push(Eigen::Vector3f(-lx, +ly, +lz) + pos);
    verts.push(Eigen::Vector3f(+lx, -ly, -lz) + pos);
    verts.push(Eigen::Vector3f(+lx, -ly, +lz) + pos);
    verts.push(Eigen::Vector3f(+lx, +ly, -lz) + pos);
    verts.push(Eigen::Vector3f(+lx, +ly, +lz) + pos);
  
    return verts;
  }

}

