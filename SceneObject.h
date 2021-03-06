
#ifndef SCENE_OBJECT_H
#define SCENE_OBJECT_H

#include <cstdio>
#include <vector>
#include <memory>

#include <eigen3/Eigen/Core>

//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include "dp_type.h"
#include "VertexObject.h"

#include "InterfaceSceneObject.h"

class SceneObject : public vaos, public/*implement*/ InterfaceSceneObject {
private:
  texture tex;

  GLint gl_tmat_loc_id_;
  GLint gl_color_loc_id_;
  GLint gl_tex_loc_id_;
  
  /* TODO integ constructor's rot/pos */
  Eigen::Matrix3d rot_; /* TODO: */
  Eigen::Vector3d pos_; /* TODO: */
  Eigen::Matrix3d scale_; /* TODO: */
  //Dp::Math::real scale_;
  Coordinates coords_;

  Eigen::Vector4f color_;

protected:
  ssg::Vertices vertices_;

public:
  /* TODO */
  SceneObject(size_t nov, GLenum target = GL_INVALID_ENUM) : 
    vaos(nov), tex(target),
    rot_(Eigen::Matrix3d::Identity()), pos_(Eigen::Vector3d::Zero())/*, scale_(1.0)*/, 
    scale_(Eigen::Matrix3d::Identity()),
    color_(0.5,0.5,0.5,1.0) {
    coords_.Rot() = Eigen::Matrix3d::Identity();
    coords_.Pos() = Eigen::Vector3d::Zero();
  }

  virtual ~SceneObject() {};

  texture& GetTexture() {
    return tex;
  }

  errno_t SetTransformMatrixLocId (GLint id) {
    if (id == -1) {
      return EINVAL;
    }

    gl_tmat_loc_id_ = id;

    return 0;
  }

  errno_t SetMaterialColorLocId (GLint id) {
    if (id == -1) {
      return EINVAL;
    }

    gl_color_loc_id_ = id;

    return 0;
  }

  errno_t SetTextureLocId (GLint id) {
    if (id == -1) {
      return EINVAL;
    }

    gl_tex_loc_id_ = id;

    return 0;
  }

  errno_t SetScale(const Eigen::Vector3d& scale) {
    scale_(0,0) = scale(0);
    scale_(1,1) = scale(1);
    scale_(2,2) = scale(2);
    return 0;
  }

  errno_t SetOffset(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
    coords_.Rot() = rot_;
    coords_.Pos() = pos_;
    rot_ = rot; /* TODO: to be deleted */
    pos_ = pos; /* TODO: to be deleted */
    return 0;
  }

  //errno_t SetScale(Dp::Math::real scale) {
  //  scale_ = scale;
  //  return 0;
  //}

  errno_t SetColor(Eigen::Vector4d &color) {
    color_ = color.cast<float>();
    return 0;
  }

  errno_t Draw(Eigen::Matrix3d& Rot, Eigen::Vector3d& Pos) {
    if (gl_tmat_loc_id_ == -1) {
      fprintf(stderr, "ERROR %s LocID(%d)\n", __PRETTY_FUNCTION__, gl_tmat_loc_id_);
      return -1;
    }

    /* TODO: why rot_ * Rot is NG? */
    //Eigen::Matrix3d rot = Rot * (rot_ * scale_);
    Eigen::Matrix3d rot = Rot * rot_ * scale_;
    //Eigen::Matrix3d rot = rot_ * Rot;
    //Eigen::Vector3d pos = Pos + Rot * scale_ * pos_;
    Eigen::Vector3d pos = Pos + Rot * pos_;

    /* TODO: move this comment.
     * OpenGL shall treat matrix as row-first rule
     * Eigen shall treat matrix as row-first rule at default 
     * So you should make transpose matrix
     */
    GLfloat transformMatrix[16] = {
      (float)rot(0,0), (float)rot(1,0), (float)rot(2,0), 0.0,
      (float)rot(0,1), (float)rot(1,1), (float)rot(2,1), 0.0,
      (float)rot(0,2), (float)rot(1,2), (float)rot(2,2), 0.0,
      (float)pos(0),   (float)pos(1)  , (float)pos(2),   1.0
    };

    /* position transformation */
    glUniformMatrix4fv(gl_tmat_loc_id_, 1, GL_FALSE, transformMatrix);
    /* material color */
    glUniform4fv(gl_color_loc_id_, 1, &color_(0));
    /* texture */
    if (vertices_.texs.size() > 0) {
      glUniform1i(gl_tex_loc_id_, tex.GetUnitNo());
      ECALL(tex.bind());
    }

    ECALL(vaos::Draw());

    if (vertices_.texs.size() > 0) {
      ECALL(tex.unbind());
    }
    return 0;
  }

  errno_t Draw() {
    if (gl_tmat_loc_id_ == -1 || gl_color_loc_id_ == -1) {
      fprintf(stderr, "ERROR %s LocID(%d)\n", __PRETTY_FUNCTION__, gl_tmat_loc_id_);
      return -1;
    }

    Eigen::Matrix3d rot = rot_ * scale_;
    Eigen::Vector3d pos = pos_;

    /* TODO: move this comment.
     * OpenGL shall treat matrix as row-first rule
     * Eigen shall treat matrix as row-first rule at default 
     * So you should make transpose matrix
     */
    GLfloat transformMatrix[16] = {
      (float)rot(0,0), (float)rot(1,0), (float)rot(2,0), 0.0,
      (float)rot(0,1), (float)rot(1,1), (float)rot(2,1), 0.0,
      (float)rot(0,2), (float)rot(1,2), (float)rot(2,2), 0.0,
      (float)pos(0),   (float)pos(1)  , (float)pos(2),   1.0
    };

    /* position transformation */
    glUniformMatrix4fv(gl_tmat_loc_id_, 1, GL_FALSE, transformMatrix);
    /* material color */
    glUniform4fv(gl_color_loc_id_, 1, &color_(0));
    /* texture */
    if (vertices_.texs.size() > 0) {
      glUniform1i(gl_tex_loc_id_, tex.GetUnitNo());
      ECALL(tex.bind());
    }

    ECALL(vaos::Draw());

    if (vertices_.texs.size() > 0) {
      ECALL(tex.unbind());
    }
    return 0;
  }

  virtual errno_t SetDrawMode(DrawMode mode) {
    fprintf(stderr, "Can not change draw mode\n");
    return -11;
  }

  virtual Coordinates& GetCoordinates() {
    return coords_;
  }

  ssg::Vertices& GetVertices() {
    return vertices_;
  }
  /* TODO */
  //std::vector<GLuint>& GetIndices() = 0;

};

#endif

