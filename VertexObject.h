
#ifndef VERTEX_OBJECT_H
#define VERTEX_OBJECT_H

#include <cmath>
#include <vector>
#include <memory>
#include <unordered_map>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <eigen3/Eigen/Core>

#include "dp_type.h"

//using namespace Eigen;

#if defined(_DEBUG)
#define DPRINTF(...) printf(__VA_ARGS__)
#else
#define DPRINTF(...)
#endif

/*
 *  pn : patitioning number
 * */

class texture {
private:
  /* GL_MAX_TEXTURE_UNITS include followings, so remove 2 from GL_MAX_TEXTURE_UNITS
   *   #define GL_ACTIVE_TEXTURE                 0x84E0
   *   #define GL_CLIENT_ACTIVE_TEXTURE          0x84E1
   */
  static constexpr GLenum kMaxUnit = ((GL_MAX_TEXTURE_UNITS - 2) - GL_TEXTURE0);

  GLuint id_;
  GLenum unitno_  = GL_INVALID_ENUM;
  GLenum target_;

  GLsizei width_;
  GLsizei height_;

  GLenum format_;
  GLenum type_;

public:

  texture(GLenum target) : target_(target), width_(0), height_(0) {
    glGenTextures(1, &id_);
    //unit_ = GL_TEXTURE0 + (id_ % kMaxUnit);
    unitno_ = (id_ % kMaxUnit);
    /* TODO: remove GL_TEXTURE_2D, use argument */
    target_ = GL_TEXTURE_2D;
    return;
  }

  virtual ~texture() {
    glDeleteTextures(1, &id_);
    DPRINTF(" glDeleteTextures: (0x%0x) ID:0x%x UNIT:0x%x\n", target_, id_, unitno_);
    id_     = GL_INVALID_VALUE;
    unitno_ = GL_INVALID_ENUM;
    return;
  }

  GLuint GetId (){
    return id_;
  }

  GLenum GetUnitId () {
    return GL_TEXTURE0 + unitno_;
  }

  GLenum GetUnitNo () {
    return unitno_;
  }

  errno_t bind () {
    glActiveTexture(GetUnitId());
    glBindTexture(target_, id_);
    DPRINTF(" glBindTexture: 0x%0x ID:0x%x UNIT:0x%x\n", target_, id_, unit_);
    return 0;
  }

  errno_t unbind (void) {
    glActiveTexture(GetUnitId());
    glBindBuffer(target_, 0);
    DPRINTF(" glBindTexture: 0x%0x ID:0x%x UNIT:0x%x -- unbind\n", target_, id_, unit_);
    return 0;
  }

  void setdata (GLsizei width, GLsizei height, GLenum format, GLenum type, const GLvoid* data) {
    bind();

    width_ = width;
    height_ = height;

    format_ = format;
    type_ = type;

    target_ = GL_TEXTURE_2D;
    glTexImage2D(GL_TEXTURE_2D, 0/*level*/,  GL_RGBA8 /*internal format*/,
    //glTexImage2D(GL_TEXTURE_2D, 0/*level*/,  GL_BGRA /*internal format*/,
    //glTexImage2D(target_, 0/*level*/, GL_COMPRESSED_RGBA/*internal format*/,
    //glTexImage2D(target_, 0/*level*/, GL_COMPRESSED_SRGB_ALPHA/*internal format*/,
                 width_, height_, 0/*border*/, format_, type_, data);
    glGenerateMipmap(target_);
  
    /* Parameters */
    //glTexParameteri(target_, GL_TEXTURE_WRAP_S, GL_REPEAT);
    //glTexParameteri(target_, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(target_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(target_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(target_, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(target_, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(target_, GL_TEXTURE_MAG_FILTER, GL_LINEAR); 

    DPRINTF(" glTexImage2D : TGT:0x%0x ID:0x%0x\n", target_, id_);

    unbind();
  }
};

class vbo {
private:
  //using namespace std;
  //using namespace Eigen;

  /* buffer id */
  GLuint buffer_id_;

  GLuint id;

  size_t noe_; /* num of elements */
  size_t data_size_;
  size_t size_;

public:
 
  vbo(GLuint buffer_id = GL_ARRAY_BUFFER) {
    buffer_id_ = buffer_id;
    glGenBuffers(1, &id);
    DPRINTF(" glGenBuffers: 0x%0x 0x%x\n", buffer_id, id);
  }

  virtual ~vbo() {
    glDeleteBuffers(1, &id);
    DPRINTF(" glDeleteBuffers: (0x%0x) 0x%x\n", buffer_id_, id);
  }

  GLuint GetId (){
    return id;
  }

  size_t size() {
    return size_;
  }

  size_t num_of_elem () {
    return noe_;
  }

  size_t data_size() {
    return data_size_;
  }

  void bind () {
    glBindBuffer(buffer_id_, id);
    DPRINTF(" glBindBuffer: 0x%0x 0x%x\n", buffer_id_, id);
    //glBindBuffer(GL_ARRAY_BUFFER, id);
  }

  void unbind (void) {
    glBindBuffer(buffer_id_, 0);
    DPRINTF(" glBindBuffers: 0x%0x 0x%x -- unbind\n", buffer_id_, id);
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  void setdata (std::vector<Eigen::Vector3f>& vertices) {
    bind();
    /* vec3 --> 3 * sizeof(GLfloat) [byte] = noe_ * sizeof(GLfloat) = data_size_ */
    size_ = vertices.size();
    noe_ = sizeof(vertices[0]) / sizeof(vertices[0][0]);
    data_size_ = sizeof(vertices[0]);

    glBufferData(buffer_id_, size_ * data_size_, &vertices[0], GL_STATIC_DRAW);
    DPRINTF(" glBufferData 3f: 0x%0x\n", buffer_id_);

    unbind();
  }

  void setdata (std::vector<Eigen::Vector2f>& normals) {
    bind();
    /* vec3 --> 3 * sizeof(GLfloat) [byte] = noe_ * sizeof(GLfloat) = data_size_ */
    size_ = normals.size();
    noe_ = sizeof(normals[0]) / sizeof(normals[0][0]);
    data_size_ = sizeof(normals[0]);

    glBufferData(buffer_id_, size_ * data_size_, &normals[0], GL_STATIC_DRAW);
    DPRINTF(" glBufferData 3f: 0x%0x\n", buffer_id_);

    unbind();
  }

  void setdata (std::vector<GLuint>& indices) {
    bind();

    size_ = indices.size();
    noe_ = sizeof(indices[0]);
    data_size_ = sizeof(indices[0]);

    glBufferData(buffer_id_, indices.size() * sizeof(indices[0]), &indices[0], GL_STATIC_DRAW);
    DPRINTF(" glBufferData ui: 0x%0x\n", buffer_id_);

    unbind();
  }
 
};


/* TODO */
#if 0
  static errno_t parseLinkAttribute (std::string &attr_type, std::ifstream &ifs, Link &link) {
      static const std::unordered_map<std::string, std::function<errno_t(std::ifstream&, Link&)>> cases = {
        {"Shape"         , [](std::ifstream &ifs, Link &link){return parseShape(ifs, link);         }},
        {"Hull"          , [](std::ifstream &ifs, Link &link){return parseHull(ifs, link);          }},
        {"Child"         , [](std::ifstream &ifs, Link &link){return parseChild(ifs, link);         }},
        {"Inertia"       , [](std::ifstream &ifs, Link &link){return parseLinkInfo(ifs, link);      }},
        {"JointInertia"  , [](std::ifstream &ifs, Link &link){return parseJointInertia(ifs, link);  }},
        {"JointViscosity", [](std::ifstream &ifs, Link &link){return parseJointViscosity(ifs, link);}},
        {"JointRange"    , [](std::ifstream &ifs, Link &link){return parseJointRange(ifs, link);    }}
      };
      //static const std::unordered_map<std::string, std::function<errno_t(std::ifstream&, Link&)>> cases = {
      //  {"JointInertia"  , parseJointInertia  },
      //  {"JointViscosity", parseJointViscosity},
      //  {"JointRange"    , parseJointRange    }
      //};
      auto result = cases.find(attr_type);
      return result != cases.end() ? result->second(ifs, link) : EINVAL ;
  }
#endif

class vao {
private:
  std::unordered_map<GLFWwindow*, GLuint> idmap;
  //GLuint id_;
  size_t num_of_data_;
  GLenum mode_;

  std::vector<std::shared_ptr<vbo>> objs;

private:
  GLuint getId () {
    auto context = glfwGetCurrentContext();
    auto result = idmap.find(context);
    if (result == idmap.end()) {
      //GLuint id;
      //glGenVertexArrays(1, &id);
      //idmap[context] = id;
      printf(" [INFO] new window(%p) try to use VAO. creating new VAO.\n", context);
      glGenVertexArrays(1, &(idmap[context]));
      drysetvbo(idmap[context]);
    }
    return idmap[context];
  }

public:

  static std::shared_ptr<GLuint> gen (size_t num) {
    std::shared_ptr<GLuint> ids(new GLuint[num], std::default_delete<GLuint[]>());
    glGenVertexArrays(num, ids.get());
    return ids;
  }

  vao(GLuint vao_id)
    : /*id_(vao_id),*/ num_of_data_(0), mode_(0) {
    idmap[glfwGetCurrentContext()] = vao_id;
  }

  vao(void)
  //vao(GLenum mode, size_t num_of_data)
    : num_of_data_(0), mode_(0)
  {
    GLuint id;
    glGenVertexArrays(1, &id);
    idmap[glfwGetCurrentContext()] = id;
    DPRINTF(" glGenVertexArrays: 0x%0x\n", id);
  }

  virtual ~vao() {
    for (auto &id : idmap) {
      DPRINTF(" glDeleteVertexArrays: 0x%x@%p cur:%p\n", id.second, id.first, glfwGetCurrentContext());
      glDeleteVertexArrays(1, &(id.second));
    }
    idmap.clear();
  }

  void setup (GLenum mode, size_t num_of_data) {
    mode_ = mode;
    num_of_data_ = num_of_data;
  }

  void bind() {
    //glBindVertexArray(id_);
    auto id = getId();
    glBindVertexArray(id);
    DPRINTF(" glBindVertexArrays: 0x%0x\n", id);
  }

  void unbind() {
    glBindVertexArray(0);
    DPRINTF(" glBindVertexArrays: 0x%0x -- unbind\n", getId());
  }

  void setmode(const GLenum mode) {
    mode_ = mode;
  }

  void drysetvbo (GLuint id) {
    glBindVertexArray(id);

    objs[0]->bind();
    objs[1]->bind();
    glVertexAttribPointer(0, objs[1]->num_of_elem(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);

    objs[2]->bind();
    glVertexAttribPointer(1, objs[2]->num_of_elem(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
  }

  errno_t setvbo(const GLint attr_idx, std::shared_ptr<vbo> obj, std::shared_ptr<vbo> subobj) {
    if (obj->size() != num_of_data_) {
      printf("ERROR2------------------ %zd vs %zd\n", obj->size(), num_of_data_);
      return EINVAL;
    }

    objs.push_back(obj);
    objs.push_back(subobj);

    bind();
    subobj->bind();
    obj->bind();


    /* enable to access VAO with in variable at shader code */
    /* 1st arg of glVertexAttribPointer is linked to 
     *   glEnableVertexAttribArray (*)
     *   glDisableVertexAttribArray(*)
     *   glBindAttribLocation      (-,*, -)
     * 第1引数はattributeの番号
     * 第2引数はattributeのサイズ
     * 第5引数は1データセットのサイズ、データセットは複数のattributeにより構成される. attributeが1つなら0でOK.
     * 第6引数は1データセット中のattributeの位置. attributeが1つなら0でOK.
     */ 
    glVertexAttribPointer(attr_idx, subobj->num_of_elem(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(attr_idx);
    DPRINTF(" glVertexAttribPointer: 0x%0x size(%zd)\n", attr_idx, subobj->num_of_elem());

    //subobj.unbind();
    //obj.unbind();
    unbind();

    return 0;
  }

  errno_t setvbo(const GLint attr_idx, std::shared_ptr<vbo> obj) {
    //if (obj->size() != num_of_data_) {
    //  printf("ERROR------------------- %zd vs %zd\n", obj->size(), num_of_data_);
    //  return EINVAL;
    //}

    objs.push_back(obj);

    bind();
    obj->bind();

    /* enable to access VAO with in variable at shader code */
    /* 1st arg of glVertexAttribPointer is linked to 
     *   glEnableVertexAttribArray (*)
     *   glDisableVertexAttribArray(*)
     *   glBindAttribLocation      (-,*, -)
     * 第1引数はattributeの番号
     * 第2引数はattributeのサイズ
     * 第5引数は1データセットのサイズ、データセットは複数のattributeにより構成される. attributeが1つなら0でOK.
     * 第6引数は1データセット中のattributeの位置. attributeが1つなら0でOK.
     */ 
    glVertexAttribPointer(attr_idx, obj->num_of_elem(), GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(attr_idx);

    obj->unbind();
    unbind();

    return 0;
  }

  errno_t Draw() {
    if (objs.size() == 0) {
      printf("ERROR 0-------------------\n");
      return EINVAL;
    }

    //if (!glIsVertexArray(id_)) {
    //  printf("UNKNOWN CONTEXT ------------------- %x\n", id_);
    //  return EINVAL;
    //}

    // TODO: 同時変換行列をuniform変数として設定する必要あり
    bind();

    // glDrawElementsの場合はどうする?
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 5);
    //glDrawArrays(mode_, 0, num_of_data_);
    glDrawElements(mode_, num_of_data_, GL_UNSIGNED_INT, 0);
    //GLuint idx[] = {0,20,1,21,2,22,3,23,4,24,5,25,6,26,7,27,8,28,9,29,10,30,11,31,12,32,13,33,14,34,15,35,16,36,17,37,18,38,19,39};
    //GLuint idx[] = {19,20,21,22,23,24,0,1,2,3,4,5};
    //glDrawElements(GL_LINES, sizeof(idx)/sizeof(idx[0]), GL_UNSIGNED_INT, (void*)idx);

    unbind();

    return 0;
  }
};

/* multiple vaos */
class vaos {
private:
  std::vector<std::shared_ptr<vao>> objs;
  //std::vector<GLuint> ids;

public:
  vaos(size_t num_of_vao) {
    objs.reserve(num_of_vao);
    std::shared_ptr<GLuint> gen_ids = vao::gen(num_of_vao);
    for (size_t i = 0; i < num_of_vao; i++) {
      objs.push_back(std::make_shared<vao>(gen_ids.get()[i]));
    }
  }

  virtual ~vaos() {
  }

  //void setup (size_t idx, GLenum mode, size_t nov) {
  //  objs[idx].setup(mode, nov);
  //}

  vao& operator[](size_t idx) {
    return *(objs[idx].get());
  }

  errno_t Draw() {
    for (auto obj : objs) {
      obj->Draw();
    }

    return 0;
  }
};

#endif
