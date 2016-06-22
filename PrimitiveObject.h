
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

class PlaneBase : public UniPartedObject {
private:
  GLfloat width_;
  GLfloat height_;

public:

  PlaneBase (
      const Eigen::Matrix3f &rot, Eigen::Vector3f pos,
      GLfloat width, GLfloat height)
    : width_(width), height_(height) {

    /* make vertices */
    vertices_ = ssg::planeVertices(Eigen::Vector3f::Zero(), width_, height_);

    vertices_.rotate(rot);
    vertices_.offset(pos);
  }

  GLfloat GetWidth()  { return width_;}
  GLfloat GetHeight() { return height_;}
};

class SolidPlane : public PlaneBase {
private:

public:
  SolidPlane (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat width, GLfloat height)
    : PlaneBase(rot, pos, width, height) {

    indices_.push_back(0);
    indices_.push_back(2);
    indices_.push_back(1);
    indices_.push_back(3);

    BuildObject();
  }
  SolidPlane (Eigen::Vector3f pos, GLfloat width, GLfloat height)
    : SolidPlane(Eigen::Matrix3f::Identity(), pos, width, height) {
  }
};

class WiredPlane : public PlaneBase {
private:

public:
  WiredPlane (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat width, GLfloat height)
    : PlaneBase(rot, pos, width, height) {

    indices_.push_back(1);
    indices_.push_back(0);
    indices_.push_back(2);

    indices_.push_back(1);
    indices_.push_back(3);
    indices_.push_back(2);

    BuildObject((const int){GL_LINE_STRIP});
  }
  WiredPlane (Eigen::Vector3f pos, GLfloat width, GLfloat height)
    : WiredPlane(Eigen::Matrix3f::Identity(), pos, width, height) {
  }
};

class RectangularBase : public UniPartedObject {
private:
  GLfloat width_;
  GLfloat length_;
  GLfloat height_;

public:

  RectangularBase (
      const Eigen::Matrix3f &rot, Eigen::Vector3f pos,
      GLfloat width, GLfloat length, GLfloat height)
    : width_(width), length_(length), height_(height) {

    /* make vertices */
    vertices_ = ssg::rectangularVertices(pos, width_, length_, height_);

    vertices_.rotate(rot);
  }

  GLfloat GetWidth()  { return width_;}
  GLfloat GetLength() { return length_;}
  GLfloat GetHeight() { return height_;}
};

class WiredRectangular : public RectangularBase {
private:

public:
  WiredRectangular (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height)
    : RectangularBase(rot, pos, width, length, height) {

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

class SolidRectangular : public RectangularBase {
private:

public:
  SolidRectangular (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height)
    : RectangularBase(rot, pos, width, length, height) {

    indices_.push_back(1);
    indices_.push_back(0);
    indices_.push_back(3);
    indices_.push_back(2);

    indices_.push_back(7);
    indices_.push_back(6);
    indices_.push_back(5);
    indices_.push_back(4);

    indices_.push_back(1);
    indices_.push_back(0);

    indices_.push_back(4);
    indices_.push_back(2);
    indices_.push_back(6);

    indices_.push_back(1);
    indices_.push_back(5);
    indices_.push_back(3);
    indices_.push_back(7);

    BuildObject();
  }
  SolidRectangular (Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height)
    : SolidRectangular(Eigen::Matrix3f::Identity(), pos, width, length, height) {
  }
};

class ConeBase : public BiPartedObject {
private:
  GLfloat radius_;
  GLfloat height_;
  GLint nop_;

public:
  ConeBase (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : radius_(radius), height_(height), nop_(nop) {

    /* make vertices */
    vertices_ = ssg::coneVertices(pos, radius_, height_, nop_);

    vertices_.rotate(rot);

  }
  ConeBase (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : ConeBase(Eigen::Matrix3f::Identity(), pos, radius, height, nop) {
  }
};

class WiredCone : public ConeBase {
private:

public:
  WiredCone (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : ConeBase(rot, pos, radius, height, nop) {

    const size_t offset = 1;

    /* make indices */
    for (size_t i = 0; i < nop + 1; i++) {
      /* top pyramid */
      indices_[kTopIdx].push_back(0);
      indices_[kTopIdx].push_back(i + offset);
     
      /* bottom circle */
      //indices_[kSidIdx].push_back(i + offset);

      /* bottom pyramid */
      indices_[kBtmIdx].push_back(i + offset);
    }

    for (size_t i = 0; i < nop + 1; i++) {
      indices_[kBtmIdx].push_back(vertices_.size() - 1);
      indices_[kBtmIdx].push_back(i + offset);
    }

    //BuildObject((const int[3]){GL_LINES, GL_LINE_STRIP, GL_LINES});
    BuildObject((const int[2]){GL_LINES, GL_LINE_STRIP});
  }

  WiredCone (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : WiredCone(Eigen::Matrix3f::Identity(), pos, radius, height, nop) {
  }
};

class SolidCone : public ConeBase {
private:

public:
  SolidCone (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t nop)
    : ConeBase(rot, pos, radius, height, nop) {

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

class SphereBase : public TriPartedObject {
private:
  GLfloat radius;
  GLint nor, noh;

public:
  SphereBase (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), nor(num_of_rpart), noh(num_of_hpart) {

    /* make vertices */
    vertices_ = ssg::sphereVertices(pos, radius, nor, noh);

    vertices_.rotate(rot);
  }

  GLfloat GetRadius() { return radius;}
  GLint GetNor() { return nor;}
  GLint GetNoh() { return noh;}
};

class WiredSphere : public SphereBase {
private:

public:
  WiredSphere (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : SphereBase(rot, pos, radius, num_of_rpart, num_of_hpart) {

    size_t nor = num_of_rpart;
    size_t noh = num_of_hpart;
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
  WiredSphere (Eigen::Vector3f pos, GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : WiredSphere(Eigen::Matrix3f::Identity(), pos, radius, num_of_rpart, num_of_hpart) {
  }
};

class SolidSphere : public SphereBase {
private:

public:
  SolidSphere (const Eigen::Matrix3f &rot, Eigen::Vector3f pos, GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : SphereBase(rot, pos, radius, num_of_rpart, num_of_hpart) {

    size_t nor = num_of_rpart;
    size_t noh = num_of_hpart;
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
  SolidSphere (Eigen::Vector3f pos, GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : SolidSphere(Eigen::Matrix3f::Identity(), pos, radius, num_of_rpart, num_of_hpart) {
  }
};

class CylinderBase : public TriPartedObject {
private:
  GLfloat radius_;
  GLfloat height_;
  size_t sectors_;

public:

  CylinderBase (
      const Eigen::Matrix3f &rot, Eigen::Vector3f pos,
      GLfloat radius, GLfloat height, size_t sectors)
    : radius_(radius), height_(height), sectors_(sectors) {

    /* make vertices */
    vertices_ = ssg::cylinderVertices(pos, radius_, height_, sectors_);

    vertices_.rotate(rot);
  }

  GLfloat GetRadius()  { return radius_;}
  GLfloat GetHeight()  { return height_;}
  GLfloat GetSectors() { return sectors_;}
};

class WiredCylinder : public CylinderBase {
private:

public:

  WiredCylinder (
      const Eigen::Matrix3f &rot, Eigen::Vector3f pos,
      GLfloat radius, GLfloat height, size_t sectors)
    : CylinderBase(rot, pos, radius, height, sectors) {

    std::cout << "ROT:" << std::endl;
    std::cout << rot << std::endl;
    std::cout << "POS:" << std::endl;
    std::cout << pos << std::endl;
    std::cout << "R:" << radius << "," << "height:" << height << "," << sectors << std::endl ;

    /* make indices */
    //const size_t num_of_slices = (vertices_.size() - 2) / 2;
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

class SolidCylinder : public CylinderBase {
private:

public:

  SolidCylinder (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors)
    : CylinderBase(rot, pos, radius, height, sectors) {

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
  SolidCylinder (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors)
   : SolidCylinder(Eigen::Matrix3f::Identity(), pos, radius, height, sectors) {
  }

};

template <class A, class B> class SwitchableSceneObject : public InterfaceSceneObject {
  InterfaceSceneObject::DrawMode mode_;
  A wired_;
  B solid_;

public:
  errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) {
    switch(mode_) {
      case WIRED: return wired_.Draw(rot, pos);
      case SOLID: return solid_.Draw(rot, pos);
      default:    return                    -1;
    }
  }

  errno_t Draw() {
    switch(mode_) {
      case WIRED: return wired_.Draw();
      case SOLID: return solid_.Draw();
      default:    return            -1;
    }
  }

  /* TODO to be removed */
  errno_t SetTransformMatrixLocId (GLint id) {
    ECALL(wired_.SetTransformMatrixLocId(id));
    ECALL(solid_.SetTransformMatrixLocId(id));
    return 0;
  }

  /* TODO to be removed */
  errno_t SetTextureLocId (GLint id) {
    ECALL(wired_.SetTextureLocId(id));
    ECALL(solid_.SetTextureLocId(id));
    return 0;
  }

  /* TODO to be removed */
  errno_t SetMaterialColorLocId (GLint id) {
    ECALL(wired_.SetMaterialColorLocId(id));
    ECALL(solid_.SetMaterialColorLocId(id));
    return 0;
  }

  errno_t SetOffset(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
    ECALL(wired_.SetOffset(pos, rot));
    ECALL(solid_.SetOffset(pos, rot));
    return 0;
  }

  //errno_t SetScale(Dp::Math::real scale) {
  //  ECALL(wired_.SetScale(scale));
  //  ECALL(solid_.SetScale(scale));
  //  return 0;
  //}

  errno_t SetColor(Eigen::Vector4d& color) {
    ECALL(wired_.SetColor(color));
    ECALL(solid_.SetColor(color));
    return 0;
  }

  SwitchableSceneObject (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat width, GLfloat height);

  SwitchableSceneObject (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height);

  SwitchableSceneObject (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors);

  errno_t SetDrawMode(DrawMode mode) {
    mode_ = mode;
    return 0;
  }

  /* TODO there are two coordinates */
  virtual Coordinates& GetCoordinates() {
    switch(mode_) {
      case WIRED: return wired_.GetCoordinates();
      case SOLID: return solid_.GetCoordinates();
      default:    return wired_.GetCoordinates();;
    }
  }
};

template<> SwitchableSceneObject<WiredPlane, SolidPlane>::SwitchableSceneObject (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat width, GLfloat height) 
    : mode_(SOLID), wired_(rot, pos, width, height), solid_(rot, pos, width, height) {
  }

template<> SwitchableSceneObject<WiredRectangular, SolidRectangular>::SwitchableSceneObject (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat width, GLfloat length, GLfloat height) 
    : mode_(SOLID), wired_(rot, pos, width, length, height), solid_(rot, pos, width, length, height) {
  }

template<> SwitchableSceneObject<WiredCylinder, SolidCylinder>::SwitchableSceneObject (const Eigen::Matrix3f& rot, Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors)
    : mode_(SOLID), wired_(rot, pos, radius, height, sectors), solid_(rot, pos, radius, height, sectors) {
  };

typedef SwitchableSceneObject<WiredPlane      , SolidPlane      > Plane;
typedef SwitchableSceneObject<WiredRectangular, SolidRectangular> Rectangular;
typedef SwitchableSceneObject<WiredCylinder   , SolidCylinder   > Cylinder;

#endif

