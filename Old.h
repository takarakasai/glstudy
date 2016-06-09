
#ifndef OLD_H
#define OLD_H

#include <stdlib.h>
#include <stdio.h>
//#include <cstdlib>
//#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <time.h> // for clock_gettime()
#include <cstring>

#include "cycle_measure.h"

#include <list>
#include <string>

#include "dp_type.h"

#include "Link.h"
#include "Shader.h"
#include "View.h"
#include "VertexObject.h"
#include "Vertex.h"
#include "Utils.h"
#include "Mesh.h"

#include "PartedObject.h"

using namespace Eigen;

static GLfloat aspect_ratio = 0;
static GLfloat size[2] = {0,0};
static GLfloat dpm = 100.0;

#define DPRINTF(...) 

class sphere : public vaos {
private:
  GLfloat radius;
  GLint nor, noh;

public:
  sphere (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), nor(num_of_rpart), noh(num_of_hpart), vaos(3) {

    ssg::Vertices verts = ssg::sphereVertices(radius, nor, noh);

    auto vert = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto norm = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    vert->setdata(verts.poses);
    norm->setdata(verts.norms);

    const size_t offset = 1;

    std::vector<GLuint> top_indices;
    std::vector<GLuint> side_indices;
    std::vector<GLuint> btm_indices;
    top_indices.push_back(0);
    btm_indices.push_back(verts.size() - 1);
    for (size_t i = 0; i < nor + 1; i++) {
      top_indices.push_back(i + offset);
      btm_indices.push_back(i + verts.size() - 1 - (noh + 1));
    }

    for (size_t i = 0; i < noh - 1; i++) {
      for (size_t j = 0; j < nor + 1; j++) {
        side_indices.push_back( i      * (nor + 1) + offset + j);
        side_indices.push_back((i + 1) * (nor + 1) + offset + j);
      }
    }

    auto tidx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    auto sidx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    auto bidx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);

    tidx->setdata(top_indices);
    sidx->setdata(side_indices);
    bidx->setdata(btm_indices);

    /* top */
    vaos::operator[](0).setup(GL_TRIANGLE_FAN, tidx->size());
    vaos::operator[](0).setvbo(0, tidx, vert);
    vaos::operator[](0).setvbo(1, norm);

    /* side */
    vaos::operator[](1).setup(GL_TRIANGLE_STRIP, sidx->size());
    vaos::operator[](1).setvbo(0, sidx, vert);
    vaos::operator[](1).setvbo(1, norm);

    /* btm */
    vaos::operator[](2).setup(GL_TRIANGLE_FAN, bidx->size());
    vaos::operator[](2).setvbo(0, bidx, vert);
    vaos::operator[](2).setvbo(1, norm);
  }
};

class cylinder : public vaos {
private:
  GLfloat radius;
  GLfloat height;
  size_t sectors;

public:

  cylinder (GLfloat radius, GLfloat height, size_t sectors)
    : radius(radius), height(height), sectors(sectors), vaos(3) {

    ssg::Vertices verts = ssg::cylinderVertices((Eigen::Vector3f){0.0, 0.0, 0.0}, radius, height, sectors);

    auto vert = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto norm = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    vert->setdata(verts.poses);
    norm->setdata(verts.norms);

    const size_t num_of_slices = (verts.size() - 2) / 2;
    const size_t offset = 2;

    std::vector<GLuint> top_indices;
    std::vector<GLuint> side_indices;
    std::vector<GLuint> btm_indices;
    top_indices.push_back(0);
    btm_indices.push_back(1);
    for (size_t i = 0; i < num_of_slices; i++) {
      top_indices.push_back(i + offset);
      side_indices.push_back(i + offset);
      side_indices.push_back(i + offset + num_of_slices);
      btm_indices.push_back(i + offset + num_of_slices);
    }

    auto tidx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    auto sidx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    auto bidx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);

    tidx->setdata(top_indices);
    sidx->setdata(side_indices);
    bidx->setdata(btm_indices);

    /* top */
    vaos::operator[](0).setup(GL_TRIANGLE_FAN, tidx->size());
    vaos::operator[](0).setvbo(0, tidx, vert);
    vaos::operator[](0).setvbo(1, norm);

    /* side */
    vaos::operator[](1).setup(GL_TRIANGLE_STRIP, sidx->size());
    vaos::operator[](1).setvbo(0, sidx, vert);
    vaos::operator[](1).setvbo(1, norm);

    /* btm */
    vaos::operator[](2).setup(GL_TRIANGLE_FAN, bidx->size());
    vaos::operator[](2).setvbo(0, bidx, vert);
    vaos::operator[](2).setvbo(1, norm);
  }

};

class sphere2 : public vaos {
private:
  GLfloat radius;
  GLint pn_r, pn_h;

public:
  sphere2 (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), pn_r(num_of_rpart), pn_h(num_of_hpart), vaos(3) {

    const GLfloat r = radius;

    /* top & bottom */
    std::vector<Eigen::Vector3f> top_vertices;
    std::vector<Eigen::Vector3f> mid_vertices;
    std::vector<Eigen::Vector3f> btm_vertices;

    top_vertices.push_back(Eigen::Vector3f(0.0, 0.0, +r));
    btm_vertices.push_back(Eigen::Vector3f(0.0, 0.0, -r));
    for (size_t i = 1; i < pn_r + 2; i++) {
      GLfloat rad = Dp::Math::PI * 1 / pn_h;
      GLfloat r_h   = r * sin(rad);
      GLfloat rad_h = 2 * Dp::Math::PI * i / pn_r;

      top_vertices.push_back(Eigen::Vector3f(
                  r_h * cos(rad_h),
                  r_h * sin(rad_h),
                  r   * cos(rad)));
      btm_vertices.push_back(Eigen::Vector3f(
                  r_h * cos(rad_h),
                  r_h * sin(rad_h),
                 -r   * cos(rad)));
    }

    for (size_t i = 1; i < pn_h - 1; i++) {
      GLfloat rad_v1 = Dp::Math::PI * (i  ) / pn_h;
      GLfloat rad_v2 = Dp::Math::PI * (i+1) / pn_h;
      GLfloat r_h1   = r * sin(rad_v1);
      GLfloat r_h2   = r * sin(rad_v2);
      GLfloat r_v1   = r * cos(rad_v1);
      GLfloat r_v2   = r * cos(rad_v2);
      for (size_t i = 0; i < pn_r + 1; i++) {
        GLfloat rad_h = 2 * Dp::Math::PI * i / pn_r;
        mid_vertices.push_back(Eigen::Vector3f(r_h2 * cos(rad_h), r_h2 * sin(rad_h), r_v2));
        mid_vertices.push_back(Eigen::Vector3f(r_h1 * cos(rad_h), r_h1 * sin(rad_h), r_v1));
      }
    }

    /* CAUTION
     * GL_QUADS & GL_QUAD_STRIP are not allowed for glDrawArrays
     * */
    //vbo top, btm, mid;
    auto top = std::make_shared<vbo>();
    auto btm = std::make_shared<vbo>();
    auto mid = std::make_shared<vbo>();
    top->setdata(top_vertices);
    vaos::operator[](0).setup(GL_TRIANGLE_FAN, num_of_rpart + 2);
    //vaos::operator[](0).setup(GL_POINTS, num_of_rpart + 2);
    vaos::operator[](0).setvbo(0, top);
    mid->setdata(mid_vertices);
    //vaos::operator[](1).setup(GL_LINES, (pn_h - 2) * (pn_r + 1) * 2);
    //vaos::operator[](1).setup(GL_POINTS, (pn_h - 2) * (pn_r + 1) * 2);
    vaos::operator[](1).setup(GL_TRIANGLE_STRIP, (pn_h - 2) * (pn_r + 1) * 2);
    //vaos::operator[](1).setup(GL_LINE_STRIP, (pn_h - 2) * (pn_r + 1) * 2);
    //vaos::operator[](1).setup(GL_TRIANGLES, (pn_h - 2) * (pn_r + 1) * 2);
    vaos::operator[](1).setvbo(0, mid);
    btm->setdata(btm_vertices);
    vaos::operator[](2).setup(GL_TRIANGLE_FAN, num_of_rpart + 2);
    //vaos::operator[](2).setup(GL_POINTS, num_of_rpart + 2);
    vaos::operator[](2).setvbo(0, btm);

    return;
  }

  virtual ~sphere2 () {
  }
};

class sphere3 : public vao {
private:
  GLfloat radius;
  GLint pn_r, pn_h;

public:
  sphere3 (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), pn_r(num_of_rpart), pn_h(num_of_hpart), vao() {

    setup(GL_TRIANGLE_FAN, num_of_rpart + 2);

    const GLfloat r = radius;

    std::vector<Eigen::Vector3f> vertices;
    vertices.push_back(Eigen::Vector3f(0.0, 0.0, r));
    for (size_t i = 1; i < pn_r + 2; i++) {
      GLfloat rad = Dp::Math::PI * 1 / pn_h;
      GLfloat r_h   = r * sin(rad);
      GLfloat rad_h = 2 * Dp::Math::PI * i / pn_r;

      vertices.push_back(Eigen::Vector3f(
                  r_h * cos(rad_h),
                  r_h * sin(rad_h),
                  r   * cos(rad)));

      //printf("%d %lf %lf %lf\n", (int)i, vertices[i][0], vertices[i][1], vertices[i][2]);
    }

    //vbo obj;
    auto obj = std::make_shared<vbo>();
    obj->setdata(vertices);
    setvbo(0, obj);

    return;
  }

  virtual ~sphere3() {
  }

};

#endif

