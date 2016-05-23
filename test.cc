#include <stdlib.h>
#include <stdio.h>
//#include <cstdlib>
//#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <time.h> // for clock_gettime()
#include <cstring>

#include "cycle_measure.h"

GLuint createProgram (const char *vsrc, const char *pv, const char *fsrc, const char *fc);

static GLfloat aspect_ratio = 0;
static GLfloat size[2] = {0,0};
static GLfloat dpm = 100.0;

#define DPRINTF(...) 

/*
** シェーダーのソースプログラムをメモリに読み込む
*/
const GLchar* readShaderSource(const char *file)
{
  FILE *fp;
  GLchar *source;
  GLsizei length;
  int ret;
  
  /* ファイルを開く */
  fp = fopen(file, "rb");
  if (fp == NULL) {
    perror(file);
    return NULL;
  }
  
  /* ファイルの末尾に移動し現在位置 (つまりファイルサイズ) を得る */
  fseek(fp, 0L, SEEK_END);
  length = ftell(fp);
  
  /* ファイルサイズのメモリを確保 */
  source = (GLchar *)malloc(length + 1);
  if (source == NULL) {
    fprintf(stderr, "Could not allocate read buffer.\n");
    return NULL;
  }
  
  /* ファイルを先頭から読み込む */
  fseek(fp, 0L, SEEK_SET);
  ret = fread((void *)source, 1, length, fp) != (size_t)length;
  fclose(fp);
  
  /* シェーダのソースプログラムのシェーダオブジェクトへの読み込み */
  if (ret) {
    fprintf(stderr, "Could not read file: %s.\n", file);
    return NULL;
  }
  //else
  //  glShaderSource(shader, 1, &source, &length);
  
  /* 確保したメモリの開放 */
  //free((void *)source);
  source[length] = '\0';
  //printf("%x %x %x\n", source[length -2], source[length - 1], source[length]);
  
  return source;
}

// シェーダのソースファイルを読み込んでプログラムオブジェクトを作成する
// vert: バーテックスシェーダのソースファイル名
// pv: バーテックスシェーダのソースプログラム中のin変数名の文字列
// frag: フラグメントシェーダのソースファイル名
// fc: フラグメントシェーダのソースプログラム中のout変数名の文字列
GLuint loadProgram (const char *vert, const char *pv, const char *frag, const char *fc) {
  //シェーダのソースファイルを読み込む
  const GLchar *vsrc = (readShaderSource(vert));
  const GLchar *fsrc = (readShaderSource(frag));
  // プログラムオブジェクトを作成する
  const GLuint program = (createProgram(vsrc, pv, fsrc, fc));
  // 読み込みに使ったメモリを解放する
  free((GLchar*)vsrc);
  free((GLchar*)fsrc);
  //delete vsrc;
  //delete fsrc;
  // 作成したプログラムオブジェクトを返す
  return program;
}

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

//シェーダオブジェクトのコンパイル結果を表示する
// shader:シェーダオブジェクト名
// str:コンパイルエラーが発生した場所を示す文字列
GLboolean printShaderInfoLog(GLuint shader, const char*str) {
  //コンパイル結果を取得する
  GLint status;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
  if (status == GL_FALSE) {
    //std::cerr << "Compile Error in " << str << std::endl;
    fprintf(stderr, "Compile Error %s\n", str);
  }
  //シェーダのコンパイル時のログの長さを取得する
  GLsizei bufSize;
  glGetShaderiv(shader, GL_INFO_LOG_LENGTH , &bufSize);
  if (bufSize > 1) {
    //シェーダのコンパイル時のログの内容を取得する
    //std::vector<GLchar> infoLog(bufSize);
    char infoLog[bufSize + 1];
    GLsizei length;
    glGetShaderInfoLog(shader, bufSize, &length, &infoLog[0]);
    //std::cerr << &infoLog[0] << std::endl;
    fprintf(stderr, "%s\n", &infoLog[0]);
  }
  //return static_cast<GLboolean>(status);
  return (GLboolean)(status);
}

//プログラムオブジェクトのリンク結果を表示する
// program:プログラムオブジェクト名
GLboolean printProgramInfoLog(GLuint program) {
  //リンク結果を取得する
  GLint status;
  glGetProgramiv(program, GL_LINK_STATUS, &status);
  if (status == GL_FALSE) {
    //std::cerr << "Link Error." << std::endl;
    fprintf(stderr, "Link Error\n");
  }
   //シェーダのリンク時のログの長さを取得する
  GLsizei bufSize;
  glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufSize);

  if (bufSize > 1) {
    //シェーダのリンク時のログの内容を取得する
    //std::vector<GLchar> infoLog(bufSize);
    char infoLog[bufSize + 1];
    GLsizei length;
    glGetProgramInfoLog(program, bufSize, &length, &infoLog[0]);
    //std::cerr << &infoLog[0] << std::endl;
    fprintf(stderr, "%s\n", &infoLog[0]);
  }
  //return static_cast<GLboolean>(status);
  return (GLboolean)(status);
}



//プログラムオブジェクトを作成する
// vsrc: バーテックスシェーダのソースプログラムの文字列
// pv: バーテックスシェーダのソースプログラム中のin変数名の文字列
// fsrc: フラグメントシェーダのソースプログラムの文字列
// fc: フラグメントシェーダのソースプログラム中のout変数名の文字列

GLuint createProgram (const char *vsrc, const char *pv, const char *fsrc, const char *fc)
{

  //空のプログラムオブジェクトを作成する
  //const GLuint program(glCreateProgram());
  const GLuint program = glCreateProgram();

  if (vsrc != NULL) {
    //バーテックスシェーダのシェーダオブジェクトを作成する
    //const GLuint vobj(glCreateShader(GL_VERTEX_SHADER));
    const GLuint vobj = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vobj, 1, &vsrc, NULL);
    glCompileShader(vobj);

    //バーテックスシェーダのシェーダオブジェクトをプログラムオブジェクトに組み込む
    if (printShaderInfoLog(vobj, "vertex shader")) {
      glAttachShader(program, vobj);
    }

    glDeleteShader(vobj);
  }

  if (fsrc != NULL) {
    //フラグメントシェーダのシェーダオブジェクトを作成する
    //const GLuint fobj(glCreateShader(GL_FRAGMENT_SHADER));
    const GLuint fobj = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(fobj, 1, &fsrc, NULL);
    glCompileShader(fobj); 

    //フラグメントシェーダのシェーダオブジェクト
    //をプログラムオブジェクトに組み込む
    if (printShaderInfoLog(fobj, "fragment shader")) {
      //glAttachShader(program, vobj);
      glAttachShader(program, fobj);
    }
    glDeleteShader(fobj);
  }

  ////
  //プログラムオブジェクトをリンクする
  glBindAttribLocation(program, 0, pv);
  glBindFragDataLocation(program, 0, fc);

  glLinkProgram(program);

  if (printProgramInfoLog(program)) {
    return program;
  }

  ////
  //作成したプログラムオブジェクトを返す
  glDeleteProgram(program);
  return 0;
}

// 頂点配列オブジェクトの作成
// vertices:頂点の数
// position:頂点の位置を格納した配列
GLuint createObject (GLuint vertices, const GLfloat (*position)[3]) {
  //頂点配列オブジェクト
  GLuint vao;
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  //頂点バッファオブジェクト
  GLuint vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof (GLfloat) * 3 * vertices, position, GL_STATIC_DRAW);
  //結合されている頂点バッファオブジェクトをin変数から参照できるようにする
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
  glEnableVertexAttribArray(0);

  //頂点配列オブジェクトの結合を解除する
  glBindVertexArray(0);
  //頂点バッファオブジェクトの結合を解除する
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  return vao;
}

#include <math.h>

void cameraMatrix(float fovy, float aspect, float near, float far, GLfloat *matrix) {
  float f = 1.0f / tanf(fovy * 0.5f * 3.141593f / 180.0f);
  float dz = far - near;
  
  matrix[ 0] = f / aspect;
  matrix[ 5] = f;
  matrix[10] = -(far + near) / dz;
  matrix[11] = -1.0f;
  matrix[14] = -2.0f * far * near / dz;
  matrix[ 1] = matrix[ 2] = matrix[ 3] = matrix[ 4] =
  matrix[ 6] = matrix[ 7] = matrix[ 8] = matrix[ 9] =
  matrix[12] = matrix[13] = matrix[15] = 0.0f;
}

void lookAt(float ex, float ey, float ez,
            float tx, float ty, float tz,
            float ux, float uy, float uz,
            GLfloat *matrix)
{
  float l;

  /* z ¼´ = e - t */
  tx = ex - tx;
  ty = ey - ty;
  tz = ez - tz;
  l = sqrtf(tx * tx + ty * ty + tz * tz);
  matrix[ 2] = tx / l;
  matrix[ 6] = ty / l;
  matrix[10] = tz / l;

  /* x ¼´ = u x z ¼´ */
  tx = uy * matrix[10] - uz * matrix[ 6];
  ty = uz * matrix[ 2] - ux * matrix[10];
  tz = ux * matrix[ 6] - uy * matrix[ 2];
  l = sqrtf(tx * tx + ty * ty + tz * tz);
  matrix[ 0] = tx / l;
  matrix[ 4] = ty / l;
  matrix[ 8] = tz / l;

  /* y ¼´ = z ¼´ x x ¼´ */
  matrix[ 1] = matrix[ 6] * matrix[ 8] - matrix[10] * matrix[ 4];
  matrix[ 5] = matrix[10] * matrix[ 0] - matrix[ 2] * matrix[ 8];
  matrix[ 9] = matrix[ 2] * matrix[ 4] - matrix[ 6] * matrix[ 0];

  /* Ê¿¹Ô°ÜÆ° */
  matrix[12] = -(ex * matrix[ 0] + ey * matrix[ 4] + ez * matrix[ 8]);
  matrix[13] = -(ex * matrix[ 1] + ey * matrix[ 5] + ez * matrix[ 9]);
  matrix[14] = -(ex * matrix[ 2] + ey * matrix[ 6] + ez * matrix[10]);

  /* »Ä¤ê */
  matrix[ 3] = matrix[ 7] = matrix[11] = 0.0f;
  matrix[15] = 1.0f;
}

void multiplyMatrix(const GLfloat *m0, const GLfloat *m1, GLfloat *matrix)
{
  for (int i = 0; i < 16; ++i) {
    int j = i & ~3, k = i & 3;

    matrix[i] = m0[j + 0] * m1[ 0 + k]
              + m0[j + 1] * m1[ 4 + k]
              + m0[j + 2] * m1[ 8 + k]
              + m0[j + 3] * m1[12 + k];
  }
}

static GLfloat projectionMatrix[16];
static GLfloat transformMatrix[16] = {
  1.0, 0.00, 0.00, 0.0,
  0.0, 0.71,-0.71, 0.0,
  0.0, 0.71, 0.71, 0.0,
  0.0, 0.00, 0.00, 1.0
};

// 形状データ
typedef struct s_Object {
  // 頂点配列オブジェクト名
  GLuint vao;
  GLuint vao2;
  // データの要素数
  GLsizei count;
}Object;

/*
 *  pn : patitioning number
 * */

typedef int32_t errno_t;

#include <eigen3/Eigen/Core>
#include <vector>
#include <memory>

#define PI 3.1415

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

    //printf("=====:%zd %zd %zd\n", size_, nov_, data_size_);
    //glBufferData(GL_ARRAY_BUFFER, size_ * data_size_, &vertices[0], GL_STATIC_DRAW);
    unbind();
  }

  void setdata (std::vector<GLuint>& indices) {
    bind();

    size_ = indices.size();
    noe_ = sizeof(indices[0]);
    data_size_ = sizeof(indices[0]);
    glBufferData(buffer_id_, indices.size() * sizeof(indices[0]), &indices[0], GL_STATIC_DRAW);
    DPRINTF(" glBufferData ui: 0x%0x\n", buffer_id_);
    //glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(indices[0]), &indices[0], GL_STATIC_DRAW);

    unbind();
  }
 

  // TODO: delete
  void setdata (std::vector<Eigen::Vector3f>& vertices, std::vector<GLuint>& indices) {
    bind();
    /* vec3 --> 3 * sizeof(GLfloat) [byte] = noe_ * sizeof(GLfloat) = data_size_ */
    size_ = vertices.size();
    noe_ = sizeof(vertices[0]) / sizeof(vertices[0][0]);
    data_size_ = sizeof(vertices[0]);
    //printf("=====:%zd %zd %zd\n", size_, nov_, data_size_);
    glBufferData(GL_ARRAY_BUFFER, size_ * data_size_, &vertices[0], GL_STATIC_DRAW);
    unbind();

    GLuint ebo;
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    DPRINTF("vbo(%u)::setdata =====:%zd %zd vs %zd\n", ebo, indices.size(), sizeof(indices[0]), vertices.size());
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(indices[0]), &indices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    unbind();
  }

};

class vao {
private:
  GLuint id;
  size_t num_of_data_;
  GLenum mode_;

  std::vector<std::shared_ptr<vbo>> objs;

public:

  static std::shared_ptr<GLuint> gen (size_t num) {
    //auto ids = std::make_shared<GLuint[num]>;
    std::shared_ptr<GLuint> ids(new GLuint[num], std::default_delete<GLuint[]>());
    glGenVertexArrays(num, ids.get());
    return std::move(ids);
  }

  vao(GLuint vao_id)
    : id(vao_id), mode_(0), num_of_data_(0){
  }

  vao(void)
  //vao(GLenum mode, size_t num_of_data)
    : mode_(0), num_of_data_(0)
  {
    glGenVertexArrays(1, &id);
    DPRINTF(" glGenVertexArrays: 0x%0x\n", id);
  }

  virtual ~vao() {
    glDeleteVertexArrays(1, &id);
    DPRINTF(" glDeleteVertexArrays: 0x%0x\n", id);
  }

  void setup (GLenum mode, size_t num_of_data) {
    mode_ = mode;
    num_of_data_ = num_of_data;
  }

  void bind() {
    glBindVertexArray(id);
    DPRINTF(" glBindVertexArrays: 0x%0x\n", id);
  }

  void unbind() {
    glBindVertexArray(0);
    DPRINTF(" glBindVertexArrays: 0x%0x -- unbind\n", id);
  }

  void setmode(const GLenum mode) {
    mode_ = mode;
  }

  errno_t setvbo(const GLint attr_idx, std::shared_ptr<vbo> obj, std::shared_ptr<vbo> subobj) {
    if (obj->size() != num_of_data_) {
      printf("ERROR------------------- %zd vs %zd\n", obj->size(), num_of_data_);
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
    if (obj->size() != num_of_data_) {
      printf("ERROR------------------- %zd vs %zd\n", obj->size(), num_of_data_);
      return EINVAL;
    }

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

  errno_t draw() {
    if (objs.size() == 0) {
      printf("ERROR 0-------------------\n");
      return EINVAL;
    }

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

  errno_t draw() {
    for (auto obj : objs) {
      obj->draw();
    }

    return 0;
  }
};

/* slices : num of divisions */
std::vector<Eigen::Vector2f> circle_tbl (size_t slices) {

  std::vector<Eigen::Vector2f> vecs;
  vecs.reserve(slices + 1);
  
  vecs.push_back(Eigen::Vector2f(1.0, 0));
  for (size_t i = 1; i < slices; i++) {
    GLfloat rad = 2 * PI * (GLfloat)i / slices;
    vecs.push_back(Eigen::Vector2f(cos(rad), sin(rad)));
    printf("%lf %lf %lf\n", cos(rad), sin(rad), rad);
  }
  vecs.push_back(Eigen::Vector2f(1.0, 0));

  return vecs;
}

class cylinder : public vaos {
private:
  GLfloat radius;
  GLfloat height;
  size_t sectors;

public:

  cylinder (GLfloat radius, GLfloat height, size_t sectors)
    : radius(radius), height(height), sectors(sectors), vaos(1) {

    const GLfloat r = radius;
    std::vector<Eigen::Vector3f> mid_vertices;
    std::vector<GLuint> mid_indices;

    auto circle  = circle_tbl(sectors);

    for (auto vec : circle) {
      auto rvec = r * vec;
      /* mid */
      mid_vertices.push_back(Eigen::Vector3f(rvec(0), rvec(1), +height/2.0));
    }

    for (auto vec : circle) {
      auto rvec = r * vec;
      /* mid */
      mid_vertices.push_back(Eigen::Vector3f(rvec(0), rvec(1), -height/2.0));
    }

    for (size_t i = 0; i < circle.size(); i++) {
      mid_indices.push_back(i);
      mid_indices.push_back(i + circle.size());
    }

    /* CAUTION
     * GL_QUADS & GL_QUAD_STRIP are not allowed for glDrawArrays
     * */
    vaos::operator[](0).setup(GL_TRIANGLE_STRIP, 2 * circle.size());

    auto mid = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto idx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    //vbo mid(GL_ARRAY_BUFFER), mid_idx(GL_ELEMENT_ARRAY_BUFFER);
    mid->setdata(mid_vertices);
    idx->setdata(mid_indices);
    vaos::operator[](0).setvbo(0, idx, mid);
    //vaos::operator[](0).setvbo(0, mid);
    //vaos::operator[](0).setvbo(0, mid_idx);
  }

#if 0
  cylinder (GLfloat radius, GLfloat height, size_t sectors)
    : radius(radius), height(height), sectors(sectors), vaos(3) {

    const GLfloat r = radius;
    std::vector<Eigen::Vector3f> top_vertices;
    std::vector<Eigen::Vector3f> mid_vertices;
    std::vector<Eigen::Vector3f> btm_vertices;

    auto circle  = circle_tbl(sectors);

    top_vertices.push_back(Eigen::Vector3f(0.0, 0.0, +height/2.0)); /* top */
    btm_vertices.push_back(Eigen::Vector3f(0.0, 0.0, -height/2.0)); /* btm */
    for (auto vec : circle) {
      auto rvec = r * vec;

      /* top */
      top_vertices.push_back(Eigen::Vector3f(rvec(0), rvec(1), +height/2.0));
      /* mid */
      mid_vertices.push_back(Eigen::Vector3f(rvec(0), rvec(1), +height/2.0));
      mid_vertices.push_back(Eigen::Vector3f(rvec(0), rvec(1), -height/2.0));
      /* btm */
      btm_vertices.push_back(Eigen::Vector3f(rvec(0), rvec(1), -height/2.0));
    }

    /* CAUTION
     * GL_QUADS & GL_QUAD_STRIP are not allowed for glDrawArrays
     * */
    vbo top, btm, mid;
    top.setdata(top_vertices);
    vaos::operator[](0).setup(GL_TRIANGLE_FAN, circle.size() + 1);
    vaos::operator[](0).setvbo(0, top);

    mid.setdata(mid_vertices);
    vaos::operator[](1).setup(GL_TRIANGLE_STRIP, 2 * circle.size());
    vaos::operator[](1).setvbo(0, mid);

    btm.setdata(btm_vertices);
    vaos::operator[](2).setup(GL_TRIANGLE_FAN, circle.size() + 1);
    vaos::operator[](2).setvbo(0, btm);
  }
#endif

};

class sphere : public vaos {
private:
  GLfloat radius;
  GLint pn_r, pn_h;

public:
  sphere (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), pn_r(num_of_rpart), pn_h(num_of_hpart), vaos(3) {

    const GLfloat r = radius;

    /* top & bottom */
    std::vector<Eigen::Vector3f> top_vertices;
    std::vector<Eigen::Vector3f> mid_vertices;
    std::vector<Eigen::Vector3f> btm_vertices;

    top_vertices.push_back(Eigen::Vector3f(0.0, 0.0, +r));
    btm_vertices.push_back(Eigen::Vector3f(0.0, 0.0, -r));
    for (size_t i = 1; i < pn_r + 2; i++) {
      GLfloat rad = PI * 1 / pn_h;
      GLfloat r_h   = r * sin(rad);
      GLfloat rad_h = 2 * PI * i / pn_r;

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
      GLfloat rad_v1 = PI * (i  ) / pn_h;
      GLfloat rad_v2 = PI * (i+1) / pn_h;
      GLfloat r_h1   = r * sin(rad_v1);
      GLfloat r_h2   = r * sin(rad_v2);
      GLfloat r_v1   = r * cos(rad_v1);
      GLfloat r_v2   = r * cos(rad_v2);
      for (size_t i = 0; i < pn_r + 1; i++) {
        GLfloat rad_h = 2 * PI * i / pn_r;
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

  virtual ~sphere () {
  }
};

class sphere2 : public vao {
private:
  GLfloat radius;
  GLint pn_r, pn_h;

public:
  sphere2 (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), pn_r(num_of_rpart), pn_h(num_of_hpart), vao() {

    setup(GL_TRIANGLE_FAN, num_of_rpart + 2);

    const GLfloat r = radius;

    std::vector<Eigen::Vector3f> vertices;
    vertices.push_back(Eigen::Vector3f(0.0, 0.0, r));
    for (size_t i = 1; i < pn_r + 2; i++) {
      GLfloat rad = PI * 1 / pn_h;
      GLfloat r_h   = r * sin(rad);
      GLfloat rad_h = 2 * PI * i / pn_r;

      vertices.push_back(Eigen::Vector3f(
                  r_h * cos(rad_h),
                  r_h * sin(rad_h),
                  r   * cos(rad)));

      printf("%d %lf %lf %lf\n", (int)i, vertices[i][0], vertices[i][1], vertices[i][2]);
    }

    //vbo obj;
    auto obj = std::make_shared<vbo>();
    obj->setdata(vertices);
    setvbo(0, obj);

    return;
  }

  virtual ~sphere2() {
  }

};

Object createShpere (GLfloat radius, GLint pn_r, GLint pn_h) {
  GLfloat r = radius;
  GLfloat top_pos[pn_r + 2][3];

  top_pos[0][0] = 0.0;
  top_pos[0][1] = 0.0;
  top_pos[0][2] =   r;

  for (size_t i = 1; i < pn_r + 2; i++) {
    GLfloat rad = PI * 1 / pn_h;
    GLfloat r_h   = r * sin(rad);
    GLfloat rad_h = 2 * PI * i / pn_r;

    top_pos[i][0] = r_h * cos(rad_h);
    top_pos[i][1] = r_h * sin(rad_h);
    top_pos[i][2] = r   * cos(rad);
    //top_pos[i][2] = r - (2.0 * r * 1 / pn_h);
    //printf("%d %lf %lf %lf\n", (int)i, top_pos[i][0], top_pos[i][1], top_pos[i][2]);
  }

  const int nov = pn_r + 2;

  Object object;
  object.vao  = createObject(nov, top_pos);
  object.count = nov;

  return object;
}

// 矩形のデータを作成する
Object createRectangle () {
  // 頂点の位置データ
  static const GLfloat position[][3] =
  {
    { -0.5, -1.5f,  1.5f },
    { -0.5, -1.5f, -1.5f },
    { -0.5,  1.5f,  1.5f },
    { -0.5,  1.5f, -1.5f }
  };
  static const GLfloat position2[][3] =
  {
    { -0.1, -1.0f,  1.0f },
    { -0.1, -1.0f, -1.0f },
    { -0.1,  1.0f,  0.5f },
    { -0.1,  1.0f, -0.5f }
  };
  // 頂点の数
  static const int vertices  = (sizeof position  / sizeof position[0]);
  static const int vertices2 = (sizeof position2 / sizeof position2[0]);
  // 頂点配列オブジェクトの作成
  Object object;
  object.vao  = createObject(vertices, position);
  object.vao2 = createObject(vertices2, position2);
  object.count = vertices;
  //object.count = vertices + vertices2;
  return object;
}

void vec3_cross_product(GLfloat a[3], GLfloat b[3], GLfloat out[3]) {

  out[0] =  a[1] * b[2] - a[2] * b[1];
  out[1] = -a[0] * b[2] + a[2] * b[0];
  out[2] =  a[0] * b[1] - a[1] * b[0];

  return;
}

void resize(GLFWwindow *const window, int width, int height)
{
  printf("--> %d %d\n", width, height);
  aspect_ratio = (GLfloat)(width) / (GLfloat)(height);
  size[0] = (GLfloat)(width);
  size[1] = (GLfloat)(height);
  glViewport(0, 0, width, height);
}

int main()
{
  GLFWwindow* window;
  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
      exit(EXIT_FAILURE);

  // OpenGL Version 3.2 Core Profileを選択する
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLfloat temp0[16];
  GLfloat temp1[16];
  cameraMatrix(90.0f, 1.0f, 0.5f, 20.0f, temp1);

  GLuint width = 640;
  GLuint height = 480;
  window = glfwCreateWindow(width, height, "Simple example", NULL, NULL);
  if (!window)
  {
      glfwTerminate();
      exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(window);

  // GLEWを初期化する
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) {
    // GLEWの初期化に失敗した
    printf("Can't initialize GLEW\n");
    return 1;
  }

  //glfwSwapInterval(1);
  //glfwSetKeyCallback(window, key_callback);

  //glViewport(100,100,640,480);

  glfwSetWindowSizeCallback(window, resize);
  resize(window, width, height);
 
  // 背景色を指定する
  glClearColor(0.0f, 0.0f, 0.2f, 0.0f);

  // hidden surface
  glEnable(GL_DEPTH_TEST);

  //プログラムオブジェクトを作成する
  //const GLuint program = createProgram(vsrc, "pv", fsrc, "fc");
  const GLuint program = loadProgram("point.vert", "pv", "point.frag", "fc");

  //const GLint aspectLoc = glGetUniformLocation(program, "aspect");
  const GLint projectionMatrixLocation = glGetUniformLocation(program, "projectionMatrix");
  const GLint transformMatrixLocation = glGetUniformLocation(program, "transformMatrix");
  const GLint sizeLoc = glGetUniformLocation(program, "size");
  const GLint dpmLoc = glGetUniformLocation(program, "dpm");
  printf("sizeLoc : %d\n", sizeLoc);
  printf("dpmLoc  : %d\n", dpmLoc);

  // make shape data
  const Object object = (createRectangle());
  
  const Object sphere1 = createShpere(0.5, 3, 3);

  //sphere sphere2(0.5, 10, 20);
  cylinder sphere2(0.5, 1.0, 20);

  GLfloat veloc = 0.05;
  GLfloat cpos[3] = {+2.0, 0.0, 0.0};
  GLfloat cdir_len = -2.0;
  GLfloat cyaw = 0.0;
  GLfloat cdir[3] = {cdir_len, 0.0, 0.0};
  GLfloat cdir_left[3];
  GLfloat cdir_to[3] = {0.0};
  GLfloat ctop_dir[3] = {0.0, 0.0, 1.0};

  cycle_measure cmeasure(5);
  cmeasure.set_cout(true);

  //ウィンドウが開いている間繰り返す
  while (glfwWindowShouldClose(window) == GL_FALSE)
  {
    static int count = 0;
    static GLfloat rad_tick = count / 100;
    GLfloat len = sqrt(pow(cdir[0], 2) + pow(cdir[1], 2) + pow(cdir[2], 2));

    vec3_cross_product(cdir, ctop_dir, cdir_left);

    if (glfwGetKey(window, GLFW_KEY_Q)) {
      break;
    }
    if (glfwGetKey(window, GLFW_KEY_PAGE_UP)) {
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
        for (size_t i = 0; i < 3; i++) {
          cpos[i] += veloc * ctop_dir[i];
        }
      } else {
        cyaw += 0.05;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN)) {
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
        for (size_t i = 0; i < 3; i++) {
          cpos[i] += -veloc * ctop_dir[i];
        }
      } else {
        cyaw -= 0.05;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_UP)) {
      for (size_t i = 0; i < 3; i++) {
        cpos[i] += veloc * cdir[i] / len;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN)) {
      for (size_t i = 0; i < 3; i++) {
        cpos[i] -= veloc * cdir[i] / len;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT)) {
      printf("%lf %lf\n", cpos[0], cpos[1]);
      // TODO: cross product
      for (size_t i = 0; i < 3; i++) {
        cpos[i] -= veloc * cdir_left[i] / len;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT)) {
      // TODO: cross product
      for (size_t i = 0; i < 3; i++) {
        cpos[i] += veloc * cdir_left[i] / len;
      }
    }

    cdir[0] = cdir_len * cos(cyaw);
    cdir[1] = cdir_len * sin(cyaw);
    for (size_t i = 0; i < 3; i++) {
      cdir_to[i] = cpos[i] + cdir[i];
    }

    //lookAt(2.0f * cos(rad_tick), 2.0f * sin(rad_tick), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, temp0);
    lookAt(cpos[0], cpos[1], cpos[2], cdir_to[0], cdir_to[1], cdir_to[2], 0.0f, 0.0f, 1.0f, temp0);
    multiplyMatrix(temp0, temp1, projectionMatrix);

    //ウィンドウを消去する
    //glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //シェーダプログラムの使用開始
    glUseProgram(program);

    glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, projectionMatrix);
    glUniformMatrix4fv(transformMatrixLocation, 1, GL_FALSE, transformMatrix);

    //glUniform1f(aspectLoc, aspect_ratio);
    glUniform2fv(sizeLoc, 1, size);
    glUniform1f(dpmLoc, dpm);

    //
    //ここで描画処理を行う
    //
    //glBindVertexArray(object.vao);
    //glDrawArrays(GL_TRIANGLE_STRIP, 0, object.count);
    //glBindVertexArray(object.vao2);
    //glDrawArrays(GL_TRIANGLE_STRIP, 0, object.count);
    //glDrawArrays(GL_QUAD_STRIP, 0, object.count);

    sphere2.draw();

    //カラーバッファを入れ替える
    glfwSwapBuffers(window);
    //glfwSwapInterval(1);

    //イベントを取り出す
#ifdef __APPLE__
    /* non block */
    glfwPollEvents();
#else
    /* non block */
    glfwWaitEvents();
#endif

    cmeasure.update();

    count++;
 }

  return 0;
}

