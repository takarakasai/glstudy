
#ifndef PARTED_OBJECT_H
#define PARTED_OBJECT_H

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
#include "Vertex.h"
#include "SceneObject.h"

class UniPartedObject : public SceneObject {
private:

protected:
  //ssg::Vertices vertices_;

  std::vector<GLuint> indices_;

public:

  /* before using BuildObject, you should prepare vertices_. */
  void BuildObject (const int draw_mode = GL_TRIANGLE_STRIP) {
    auto vert = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto norm = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    vert->setdata(vertices_.poses);
    norm->setdata(vertices_.norms);

    auto idx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    idx->setdata(indices_);

    vaos::operator[](0).setup(draw_mode, idx->size());
    vaos::operator[](0).setvbo(0, idx, vert);
    vaos::operator[](0).setvbo(1, norm);
    if (vertices_.texs.size() > 0) {
      auto tex = std::make_shared<vbo>(GL_ARRAY_BUFFER);
      tex->setdata(vertices_.texs);
      vaos::operator[](0).setvbo(2, tex);
    }
  };

  UniPartedObject() : SceneObject(1, GL_TEXTURE_2D) {};
  virtual ~UniPartedObject() {};

  std::vector<GLuint>& GetIndices() {
    return indices_;
  }
};

class BiPartedObject : public SceneObject {
private:

protected:
  //ssg::Vertices vertices_;

  static const size_t kTopIdx = 0;
  static const size_t kBtmIdx = 1;
  static const size_t kNofIdx = 2;
  std::vector<GLuint> indices_[kNofIdx];

public:

  /* before using BuildObject, you should prepare vertices_. */
  void BuildObject (const int draw_mode[kNofIdx] = (int[kNofIdx]){GL_TRIANGLE_FAN, GL_TRIANGLE_FAN}) {

    auto vert = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto norm = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    vert->setdata(vertices_.poses);
    norm->setdata(vertices_.norms);

    std::shared_ptr<vbo> vbo_indices[kNofIdx];

    size_t i = 0;
    for (auto idx : vbo_indices) {
      idx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);

      idx->setdata(indices_[i]);
      vaos::operator[](i).setup(draw_mode[i], idx->size());
      vaos::operator[](i).setvbo(0, idx, vert);
      vaos::operator[](i).setvbo(1, norm);

      i++;
    }
  };

  std::vector<GLuint>& GetIndices() {
   /* TODO currently only first of indices will return. */
    return indices_[0];
  }

  BiPartedObject() : SceneObject(kNofIdx) {};
  virtual ~BiPartedObject() {};
};

class TriPartedObject : public SceneObject {
private:

protected:
  //ssg::Vertices vertices_;

  static const size_t kTopIdx = 0;
  static const size_t kSidIdx = 1;
  static const size_t kBtmIdx = 2;
  static const size_t kNofIdx = 3;
  std::vector<GLuint> indices_[kNofIdx];

public:

  /* before using BuildObject, you should prepare vertices_. */
  void BuildObject (const int draw_mode[3] = (int[3]){GL_TRIANGLE_FAN, GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN}) {

    auto vert = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto norm = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    vert->setdata(vertices_.poses);
    norm->setdata(vertices_.norms);

    std::shared_ptr<vbo> vbo_indices[kNofIdx];

    size_t i = 0;
    for (auto idx : vbo_indices) {
      idx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);

      idx->setdata(indices_[i]);
      vaos::operator[](i).setup(draw_mode[i], idx->size());
      vaos::operator[](i).setvbo(0, idx, vert);
      vaos::operator[](i).setvbo(1, norm);

      i++;
    }
  };

  std::vector<GLuint>& GetIndices() {
   /* TODO currently only first of indices will return. */
    return indices_[0];
  }

  TriPartedObject() : SceneObject(3) {};
  virtual ~TriPartedObject() {};
};

#endif

