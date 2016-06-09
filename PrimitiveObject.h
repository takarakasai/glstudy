
#ifndef PRIMITIVE_OBJECT_H
#define PRIMITIVE_OBJECT_H

#include <stdlib.h>
#include <stdio.h> //#include <cstdlib>
//#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <time.h> // for clock_gettime()
#include <cstring>

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

#include <math.h>

class WiredRectangular : public UniPartedObject {
private:
  //Vector3f center_;
  GLfloat width_;
  GLfloat length_;
  GLfloat height_;

public:
  WiredRectangular (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height)
    : width_(width), length_(length), height_(height) {

    /* make vertices */
    vertices_ = ssg::rectangularVertices(pos, width_, length_, height_);

    vertices_.rotate(rot);

    indices_.push_back(0); indices_.push_back(1);
    indices_.push_back(0); indices_.push_back(2);
    indices_.push_back(0); indices_.push_back(4);

    indices_.push_back(3); indices_.push_back(1);
    indices_.push_back(3); indices_.push_back(2);
    indices_.push_back(3); indices_.push_back(7);

    indices_.push_back(5); indices_.push_back(1);
    indices_.push_back(5); indices_.push_back(4);
    indices_.push_back(5); indices_.push_back(7);

    indices_.push_back(6); indices_.push_back(2);
    indices_.push_back(6); indices_.push_back(4);
    indices_.push_back(6); indices_.push_back(7);

    BuildObject((const int){GL_LINES});
  }
  WiredRectangular (Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height)
    : WiredRectangular(Eigen::Matrix3f::Identity(), pos, width, length, height) {
  }
};

class WiredCone : public TriPartedObject {
private:
  GLfloat radius_;
  GLfloat height_;
  GLint nop_;

public:
  WiredCone (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : radius_(radius), height_(height), nop_(nop) {

    /* make vertices */
    vertices_ = ssg::coneVertices(pos, radius_, height_, nop_);

    vertices_.rotate(rot);

    const size_t offset = 1;

    /* make indices */
    for (size_t i = 0; i < nop + 1; i++) {
      /* top pyramid */
      indices_[kTopIdx].push_back(0);
      indices_[kTopIdx].push_back(i + offset);
     
      /* bottom circle */
      indices_[kSidIdx].push_back(i + offset);

      /* bottom pyramid */
      indices_[kBtmIdx].push_back(vertices_.size() - 1);
      indices_[kBtmIdx].push_back(i + offset);
    }

    BuildObject((const int[3]){GL_LINES, GL_LINE_STRIP, GL_LINES});
  }
  WiredCone (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : WiredCone(Eigen::Matrix3f::Identity(), pos, radius, height, nop) {
  }
};

class SolidCone : public BiPartedObject {
private:
  GLfloat radius_;
  GLfloat height_;
  GLint nop_;

public:
  SolidCone (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : radius_(radius), height_(height), nop_(nop) {

    /* make vertices */
    vertices_ = ssg::coneVertices(pos, radius_, height_, nop_);

    vertices_.rotate(rot);

    const size_t offset = 1;

    indices_[kTopIdx].push_back(0);
    indices_[kBtmIdx].push_back(vertices_.size() - 1);

    /* make indices */
    for (size_t i = 0; i < nop + 1; i++) {
      /* top pyramid */
      indices_[kTopIdx].push_back(i + offset);

      /* bottom pyramid */
      indices_[kBtmIdx].push_back(i + offset);
    }

    BuildObject();
  }
  SolidCone (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : SolidCone(Eigen::Matrix3f::Identity(), pos, radius, height, nop) {
  }
};

class WiredSphere : public TriPartedObject {
private:
  GLfloat radius;
  GLint nor, noh;

public:
  WiredSphere (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), nor(num_of_rpart), noh(num_of_hpart) {

    /* make vertices */
    vertices_ = ssg::sphereVertices(radius, nor, noh);

    const size_t offset = 1;

    /* make indices */
    for (size_t i = 0; i < nor + 1; i++) {
      /* top pyramid */
      indices_[kTopIdx].push_back(0);
      indices_[kTopIdx].push_back(i + offset);

      /* top horizonal circle */
      indices_[kSidIdx].push_back(i + offset);

      /* bottom pyramid */
      indices_[kBtmIdx].push_back(    vertices_.size() - 1);
      indices_[kBtmIdx].push_back(i + vertices_.size() - 1 - (nor + 1));
    }

    for (size_t i = 0; i < noh - 2; i++) {
      for (size_t j = 0; j < nor + 1; j++) {
        /* lines between every sequential two horizonal circles */
        indices_[kSidIdx].push_back( i      * (nor + 1) + offset + j);
        indices_[kSidIdx].push_back((i + 1) * (nor + 1) + offset + j);
      }
      for (size_t j = 0; j < nor + 1; j++) {
        /* other horizonal circles */
        indices_[kSidIdx].push_back((i + 1) * (nor + 1) + offset + j);
      }
    }

    BuildObject((const int[3]){GL_LINES, GL_LINE_STRIP, GL_LINES});
  }
};

class SolidSphere : public TriPartedObject {
private:
  GLfloat radius;
  GLint nor, noh;

public:
  SolidSphere (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), nor(num_of_rpart), noh(num_of_hpart) {

    /* make vertices */
    vertices_ = ssg::sphereVertices(radius, nor, noh);

    const size_t offset = 1;

    /* make indices */
    indices_[kTopIdx].push_back(0);
    indices_[kBtmIdx].push_back(vertices_.size() - 1);
    for (size_t i = 0; i < nor + 1; i++) {
      indices_[kTopIdx].push_back(i + offset);
      indices_[kBtmIdx].push_back(i + vertices_.size() - 1 - (nor + 1));
    }

    for (size_t i = 0; i < noh - 1; i++) {
      for (size_t j = 0; j < nor + 1; j++) {
        indices_[kSidIdx].push_back( i      * (nor + 1) + offset + j);
        indices_[kSidIdx].push_back((i + 1) * (nor + 1) + offset + j);
      }
    }

    BuildObject();
  }
};

class WiredCylinder : public TriPartedObject {
private:
  GLfloat radius;
  GLfloat height;
  size_t sectors;

public:

  WiredCylinder (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors)
  //WiredCylinder (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors) 
    : radius(radius), height(height), sectors(sectors) {

    std::cout << "ROT:" << std::endl;
    std::cout << rot << std::endl;
    std::cout << "POS:" << std::endl;
    std::cout << pos << std::endl;
    std::cout << "R:" << radius << "," << "height:" << height << "," << sectors << std::endl ;

    /* make vertices */
    vertices_ = ssg::cylinderVertices(pos, radius, height, sectors);
   
    //Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
    //Eigen::Matrix3f rot;
    vertices_.rotate(rot);

    /* make indices */
    const size_t num_of_slices = (vertices_.size() - 2) / 2;
    const size_t offset = 2;

    /* make indices */
    for (size_t i = 0; i < sectors + 1; i++) {
      /* top pyramid */
      indices_[kTopIdx].push_back(0);
      indices_[kTopIdx].push_back(i + offset);

      /* top horizonal circle */
      indices_[kSidIdx].push_back(i + offset);

      /* bottom pyramid */
      indices_[kBtmIdx].push_back(1);
      indices_[kBtmIdx].push_back(i + offset + (sectors + 1));
    }

    for (size_t i = 0; i < sectors + 1; i++) {
      indices_[kSidIdx].push_back(i + offset);
      indices_[kSidIdx].push_back(i + offset + (sectors + 1));
    }

    for (size_t i = 0; i < sectors + 1; i++) {
      indices_[kSidIdx].push_back(i + offset + (sectors + 1));
    }

    BuildObject((const int[3]){GL_LINES, GL_LINE_STRIP, GL_LINES});
  }

  WiredCylinder (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors)
   : WiredCylinder(Eigen::Matrix3f::Identity(), pos, radius, height, sectors) {
  }
};

class SolidCylinder : public TriPartedObject {
private:
  GLfloat radius;
  GLfloat height;
  size_t sectors;

public:

  SolidCylinder (GLfloat radius, GLfloat height, size_t sectors)
    : radius(radius), height(height), sectors(sectors) {

    /* make vertices */
    vertices_ = ssg::cylinderVertices((Eigen::Vector3f){0.0, 0.0, 0.0}, radius, height, sectors);

    /* make indices */
    const size_t num_of_slices = (vertices_.size() - 2) / 2;
    const size_t offset = 2;

    indices_[kTopIdx].push_back(0);
    indices_[kBtmIdx].push_back(1);
    for (size_t i = 0; i < num_of_slices; i++) {
      indices_[kTopIdx].push_back(i + offset);
      indices_[kSidIdx].push_back(i + offset);
      indices_[kSidIdx].push_back(i + offset + num_of_slices);
      indices_[kBtmIdx].push_back(i + offset + num_of_slices);
    }

    BuildObject();
  }
};

#endif

