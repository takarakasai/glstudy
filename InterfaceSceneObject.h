
#ifndef INTERFACE_SCENE_OBJECT_H
#define INTERFACE_SCENE_OBJECT_H

#include <vector>

#include <eigen3/Eigen/Core>

/* TODO:remove */
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "dp_type.h"
#include "CasCoords.h"
/* TODO shall be removed */
#include "Vertex.h"

// interface
class InterfaceSceneObject {
private:
public:
  virtual errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) = 0;
  virtual errno_t Draw() = 0;

  /* TODO: scene class should have those setter */
  virtual errno_t SetTransformMatrixLocId (GLint id) = 0;
  virtual errno_t SetTextureLocId (GLint id) = 0;
  virtual errno_t SetMaterialColorLocId (GLint id) = 0;
  //errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) {return 0;};
  //errno_t SetTransformMatrixLocId (GLint id) {return 0;};

  /* TODO */
  virtual errno_t SetScale(const Eigen::Vector3d& scale) = 0;
  virtual errno_t SetOffset(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) = 0;
  virtual errno_t SetColor(Eigen::Vector4d& color) = 0;
  //virtual errno_t SetScale(Dp::Math::real scale) = 0;

  typedef enum {
    WIRED,
    SOLID
  } DrawMode;
  virtual errno_t SetDrawMode(DrawMode mode) = 0;
  virtual Coordinates& GetCoordinates() = 0;

  /* for collision mesh data */
  virtual ssg::Vertices& GetVertices() = 0;
  virtual std::vector<GLuint>& GetIndices() = 0;

  //InterfaceSceneObject() {};
  virtual ~InterfaceSceneObject() {};
};

#endif

