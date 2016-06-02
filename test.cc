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

GLuint createProgram (const char *vsrc, const std::list<std::string> attrs, const char *fsrc, const char *fc);

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
GLuint loadProgram (const char *vert, const std::list<std::string> &attrs, const char *frag, const char *fc) {
  //シェーダのソースファイルを読み込む
  const GLchar *vsrc = (readShaderSource(vert));
  const GLchar *fsrc = (readShaderSource(frag));
  // プログラムオブジェクトを作成する
  const GLuint program = (createProgram(vsrc, attrs, fsrc, fc));
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

GLuint createProgram (const char *vsrc, const std::list<std::string> attrs, const char *fsrc, const char *fc)
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
  GLuint idx = 0;
  for (auto &attr : attrs) {
    glBindAttribLocation(program, idx++, attr.c_str());
    printf("glBindAttribLocation: %u %u %s\n", program, idx, attr.c_str());
  }
  //glBindAttribLocation(program, 0, pv);
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

#include <vector>
#include <memory>
#include <eigen3/Eigen/Core>

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
 

#if 0
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
#endif

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

/* slices : num of divisions */
std::vector<Eigen::Vector2f> circle_tbl (size_t slices) {

  std::vector<Eigen::Vector2f> vecs;
  vecs.reserve(slices + 1);
  
  vecs.push_back(Eigen::Vector2f(1.0, 0));
  for (size_t i = 1; i < slices; i++) {
    GLfloat rad = 2 * Dp::Math::PI * (GLfloat)i / slices;
    vecs.push_back(Eigen::Vector2f(cos(rad), sin(rad)));
    //printf("%lf %lf %lf\n", cos(rad), sin(rad), rad);
  }
  vecs.push_back(Eigen::Vector2f(1.0, 0));

  return vecs;
}

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

Vertices cylinderVertices (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors) {
  Vertices verts;

  const GLfloat r = radius;
  auto circle  = circle_tbl(sectors);

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

Vertices sphereVertices (GLfloat radius, size_t nor, size_t noh) {
  Vertices verts;

  const GLfloat r = radius;
  auto circle  = circle_tbl(nor);

  verts.push(Eigen::Vector3f(0, 0, +r));

  for (size_t i = 1; i < noh; i++) {
    auto rad = Dp::Math::PI * i / noh;
    auto h   = r * cos(rad);
    auto rv  = r * sin(rad);

    for (auto vec : circle) {
      auto rvec = rv * vec;
      verts.push(Eigen::Vector3f(rvec(0), rvec(1), h));
    }
  }

  verts.push(Eigen::Vector3f(0, 0, -r));

  return verts;
}

Vertices coneVertices (Eigen::Vector3f pos, GLfloat radius, GLfloat height, size_t sectors) {
  Vertices verts;

  const GLfloat r = radius;
  auto circle  = circle_tbl(sectors);

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

#define ECALL(function)     \
  do {                      \
    errno_t eno = function; \
    if (eno != 0) {         \
      return eno;           \
    }                       \
  } while(0)

// interface
class InterfaceSceneObject {
private:
public:
  virtual errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) = 0;
  virtual errno_t SetTransformMatrixLocId (GLint id) = 0;
  //errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) {return 0;};
  //errno_t SetTransformMatrixLocId (GLint id) {return 0;};

  /* TODO */
  virtual errno_t SetOffset(Eigen::Vector3d& pos, Eigen::Matrix3d& rot) = 0;
  virtual errno_t SetScale(Dp::Math::real scale) = 0;

  //InterfaceSceneObject() {};
  virtual ~InterfaceSceneObject() {};
};

//#include <eigen3/Eigen/Geometry>
#include "dp_type.h"

class SceneObject : public vaos, public/*implement*/ InterfaceSceneObject {
private:
  GLint gl_tmat_loc_id_;
  
  /* TODO integ constructor's rot/pos */
  Eigen::Matrix3d rot_;
  Eigen::Vector3d pos_;
  Dp::Math::real scale_;

public:
  SceneObject(size_t nov) : vaos(nov), rot_(Eigen::Matrix3d::Identity()), pos_(Eigen::Vector3d::Zero()), scale_(1.0) {
    gl_tmat_loc_id_ = 0;
  };

  virtual ~SceneObject() {};

  errno_t SetTransformMatrixLocId (GLint id) {
    if (gl_tmat_loc_id_ == -1) {
      return EINVAL;
    }

    gl_tmat_loc_id_ = id;

    return 0;
  }

  errno_t SetOffset(Eigen::Vector3d& pos, Eigen::Matrix3d& rot) {
    rot_ = rot;
    pos_ = pos;
  }

  errno_t SetScale(Dp::Math::real scale) {
    scale_ = scale;
  }

  errno_t Draw(Eigen::Matrix3d& Rot, Eigen::Vector3d& Pos) {
    if (gl_tmat_loc_id_ == -1) {
      fprintf(stderr, "ERROR %s (%d)\n", __PRETTY_FUNCTION__, gl_tmat_loc_id_);
      return -1;
    }

    /* TODO: why rot_ * Rot is NG? */
    Eigen::Matrix3d rot = Rot * (rot_ * scale_);
    //Eigen::Matrix3d rot = rot_ * Rot;
    Eigen::Vector3d pos = Pos + Rot * pos_;

    //GLfloat transformMatrix[16] = {
    //  (float)rot(0,0), (float)rot(0,1), (float)rot(0,2), 0.0,
    //  (float)rot(1,0), (float)rot(1,1), (float)rot(1,2), 0.0,
    //  (float)rot(2,0), (float)rot(2,1), (float)rot(2,2), 0.0,
    //  (float)pos(0),   (float)pos(1)  , (float)pos(2),   1.0
    //};
    GLfloat transformMatrix[16] = {
      (float)rot(0,0), (float)rot(1,0), (float)rot(2,0), 0.0,
      (float)rot(0,1), (float)rot(1,1), (float)rot(2,1), 0.0,
      (float)rot(0,2), (float)rot(1,2), (float)rot(2,2), 0.0,
      (float)pos(0),   (float)pos(1)  , (float)pos(2),   1.0
    };
 
    //  1.0, 0.00, 0.00, 0.0,
    //  0.0, 0.71,-0.71, 0.0,
    //  0.0, 0.71, 0.71, 0.0,
    //  0.0, 0.00, 1.00, 1.0
    //};



//    for (size_t i = 0; i < 4; i++) {
//      printf("%+7.2f %+7.2f %+7.2f %+7.2f\n", transformMatrix[i*4], transformMatrix[i*4+1], transformMatrix[i*4+2], transformMatrix[i*4+3]);
//    }

    //  1.0, 0.00, 0.00, 0.0,
    //  0.0, 0.71,-0.71, 0.0,
    //  0.0, 0.71, 0.71, 0.0,
    //  0.0, 0.00, 0.00, 1.0
    //};

    glUniformMatrix4fv(gl_tmat_loc_id_, 1, GL_FALSE, transformMatrix);

    ECALL(vaos::Draw());
  }
};

class UniPartedObject : public SceneObject {
private:

protected:
  Vertices vertices_;

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
  };

  UniPartedObject() : SceneObject(1) {};
  virtual ~UniPartedObject() {};
};

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <functional>

namespace ssg {
  class UniMesh : public UniPartedObject {
  private:
  
  public:
    UniMesh (const Eigen::Matrix3f &rot, const aiMesh* paiMesh) {
      /* make vertices */
      const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
      for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
        const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
        const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);
        const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;
 
        vertices_.push(Eigen::Vector3f(pPos->x, pPos->y, pPos->z),
                   Eigen::Vector3f(pNormal->x, pNormal->y, pNormal->z));
        //printf("%+7.2lf, %+7.2lf, %+7.2lf : %+7.2lf, %+7.2lf || %+7.2lf, %+7.2lf, %+7.2lf -- %s\n",
        //  pPos->x, pPos->y, pPos->z, pTexCoord->x, pTexCoord->y, pNormal->x, pNormal->y, pNormal->z,
        //  paiMesh->HasTextureCoords(0) ? "True" : "False");
      }
      vertices_.rotate(rot);
  
      /* make indices */
      for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        //printf("%d, %d, %d\n", Face.mIndices[0], Face.mIndices[1], Face.mIndices[2]);
  
        indices_.push_back(Face.mIndices[0]);
        indices_.push_back(Face.mIndices[1]);
        indices_.push_back(Face.mIndices[2]);
      }
  
      BuildObject();
    }

    UniMesh (const aiMesh* paiMesh) :UniMesh(Eigen::Matrix3f::Identity(), paiMesh){}
  };

  std::shared_ptr<SceneObject> ImportObject (std::string file_name) {
    Assimp::Importer Importer;
  
    const aiScene* pScene = 
      Importer.ReadFile(file_name.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
    // aiProcess_FlipUVs
  
    if (pScene == NULL) {
      printf("Error parsing '%s': '%s'\n", file_name.c_str(), Importer.GetErrorString());
      return NULL;
    }

    printf(" mesh size : %u\n", pScene->mNumMeshes);
    printf(" mate size : %u\n", pScene->mNumMaterials);

    if (pScene->mNumMeshes == 1) {
      printf("NumMeshes == 1 ==> UniMesh\n");
      return std::make_shared<UniMesh>(pScene->mMeshes[0]);
    }

    printf("NumMeshes > 1 not supported\n");
    return NULL;
  }

  void test() {
    Assimp::Importer Importer;
  
    std::string Filename = "./test.stl";
    printf("------------ %s\n", Filename.c_str());
  
    const aiScene* pScene = 
      Importer.ReadFile(Filename.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
    // aiProcess_FlipUVs
  
    DPRINTF("result '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
    if (pScene) {
      //Ret = InitFromScene(pScene, Filename);
      printf("result '%s': \n", Filename.c_str());
    }
    else {
      printf("Error parsing '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
      return;
    }
  
    printf(" mesh size : %u\n", pScene->mNumMeshes);
    printf(" mate size : %u\n", pScene->mNumMaterials);
  
    const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
   
    for (unsigned int i = 0 ; i < pScene->mNumMeshes ; i++) {
      const aiMesh* paiMesh = pScene->mMeshes[i];
      //InitMesh(i, paiMesh);
  
      for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
        const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
        const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);
        const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;
  
        printf("%+7.2lf, %+7.2lf, %+7.2lf : %+7.2lf, %+7.2lf || %+7.2lf, %+7.2lf, %+7.2lf\n",
          pPos->x, pPos->y, pPos->z, pTexCoord->x, pTexCoord->y, pNormal->x, pNormal->y, pNormal->z);
        //Vertex v(Vector3f(pPos->x, pPos->y, pPos->z),
        //         Vector2f(pTexCoord->x, pTexCoord->y),
        //         Vector3f(pNormal->x, pNormal->y, pNormal->z));
        //
        //Vertices.push_back(v);
      }
  
      for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        printf("%d, %d, %d\n", Face.mIndices[0], Face.mIndices[1], Face.mIndices[2]);
        //Indices.push_back(Face.mIndices[0]);
        //Indices.push_back(Face.mIndices[1]);
        //Indices.push_back(Face.mIndices[2]);
      }
    }
  
    //return InitMaterials(pScene, Filename);
  }
}

class BiPartedObject : public SceneObject {
private:

protected:
  Vertices vertices_;

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

  BiPartedObject() : SceneObject(kNofIdx) {};
  virtual ~BiPartedObject() {};
};

class TriPartedObject : public SceneObject {
private:

protected:
  Vertices vertices_;

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

  TriPartedObject() : SceneObject(3) {};
  virtual ~TriPartedObject() {};
};

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
    vertices_ = rectangularVertices(pos, width_, length_, height_);

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
    vertices_ = coneVertices(pos, radius_, height_, nop_);

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
    vertices_ = coneVertices(pos, radius_, height_, nop_);

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
    vertices_ = sphereVertices(radius, nor, noh);

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
    vertices_ = sphereVertices(radius, nor, noh);

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

class sphere : public vaos {
private:
  GLfloat radius;
  GLint nor, noh;

public:
  sphere (GLfloat radius, GLint num_of_rpart, GLint num_of_hpart)
    : radius(radius), nor(num_of_rpart), noh(num_of_hpart), vaos(3) {

    Vertices verts = sphereVertices(radius, nor, noh);

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
    vertices_ = cylinderVertices(pos, radius, height, sectors);
   
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
    vertices_ = cylinderVertices((Eigen::Vector3f){0.0, 0.0, 0.0}, radius, height, sectors);

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

class cylinder : public vaos {
private:
  GLfloat radius;
  GLfloat height;
  size_t sectors;

public:

  cylinder (GLfloat radius, GLfloat height, size_t sectors)
    : radius(radius), height(height), sectors(sectors), vaos(3) {

    Vertices verts = cylinderVertices((Eigen::Vector3f){0.0, 0.0, 0.0}, radius, height, sectors);

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

#if 0
  cylinder (GLfloat radius, GLfloat height, size_t sectors)
    : radius(radius), height(height), sectors(sectors), vaos(1) {

    const GLfloat r = radius;
    std::vector<Eigen::Vector3f> mid_vertices;
    std::vector<Eigen::Vector3f> mid_normals;
    std::vector<GLuint> mid_indices;

    auto circle  = circle_tbl(sectors);

    for (auto vec : circle) {
      auto rvec = r * vec;
      /* mid */
      Eigen::Vector3f pvec(rvec(0), rvec(1), +height/2.0);
      auto nvec = pvec / pvec.norm();
      mid_vertices.push_back(pvec);
      mid_normals.push_back(nvec);
    }

    for (auto vec : circle) {
      auto rvec = r * vec;
      /* mid */
      Eigen::Vector3f pvec(rvec(0), rvec(1), -height/2.0);
      auto nvec = pvec / pvec.norm();
      mid_vertices.push_back(pvec);
      mid_normals.push_back(nvec);
    }

    for (size_t i = 0; i < circle.size(); i++) {
      mid_indices.push_back(i);
      mid_indices.push_back(i + circle.size());
    }

    /* CAUTION
     * GL_QUADS & GL_QUAD_STRIP are not allowed for glDrawArrays
     * */
    vaos::operator[](0).setup(GL_TRIANGLE_STRIP, 2 * circle.size());

    auto vert = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto norm = std::make_shared<vbo>(GL_ARRAY_BUFFER);
    auto idx = std::make_shared<vbo>(GL_ELEMENT_ARRAY_BUFFER);
    vert->setdata(mid_vertices);
    norm->setdata(mid_normals);
    idx->setdata(mid_indices);
    vaos::operator[](0).setvbo(0, idx, vert);
    vaos::operator[](0).setvbo(1, norm);
  }
#endif

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

#include "CasCoords.h"

#include <eigen3/Eigen/Geometry>

namespace Eigen {
  typedef Matrix< double, 6, 1 >  Vector6d;
  typedef Matrix< double, 6, 6 >  Matrix6d;
}

class Actuator {
private:
  Dp::Math::real& value_;
  Dp::Math::real& veloc_;
  Dp::Math::real& accel_;
  Dp::Math::real& acfrc_; /* actuational force */

  Dp::Math::real& exfrc_; /* external force */

public:
  Actuator(Dp::Math::real& value,
           Dp::Math::real& veloc,
           Dp::Math::real& accel,
           Dp::Math::real& acfrc,
           Dp::Math::real& exfrc
          ) : value_(value), veloc_(veloc), accel_(accel),
              acfrc_(acfrc), exfrc_(exfrc) {};
  virtual ~Actuator() {};

  inline const Dp::Math::real& GetValue()    { return value_;};
  inline const Dp::Math::real& GetVeloc()    { return veloc_;};
  inline const Dp::Math::real& GetAccel()    { return accel_;};
  inline const Dp::Math::real& GetActForce() { return acfrc_;};
  inline const Dp::Math::real& GetExtForce() { return exfrc_;};

  inline void SetValue(Dp::Math::real& value)    { value_ = value;};
  inline void SetVeloc(Dp::Math::real& veloc)    { veloc_ = veloc;};
  inline void SetAccel(Dp::Math::real& accel)    { accel_ = accel;};
  inline void SetActForce(Dp::Math::real& force) { acfrc_ = force;};
};

#include <iostream>

//class Joint : public CasCoords {
class Joint {
private:
  //const char* name_;
  std::string name_;

protected:
  std::list<std::shared_ptr<Actuator>> actuators_;

  /* configuration */
  Eigen::Vector6d inertia_;
  Eigen::Vector6d viscosity_;
  Eigen::Vector6d min_val_;
  Eigen::Vector6d max_val_;

  Eigen::Vector6d val_;
  Eigen::Vector6d vel_;
  Eigen::Vector6d acc_;
  Eigen::Vector6d efrc_; /* external torque */

  Eigen::Vector6d afrc_; /* actuation torque */

  /* temporary rot/pos */
  Eigen::Matrix3d rot_;
  Eigen::Vector3d pos_;

public:
  Joint(const char* name) : name_(name) {
    rot_ = Eigen::Matrix3d::Identity();
    pos_ = Eigen::Vector3d::Zero();
  };
  virtual ~Joint() {};

  void SetInertia(size_t idx, Dp::Math::real val) {
    inertia_(idx) = val;
  }

  void SetViscosity(size_t idx, Dp::Math::real val) {
    viscosity_(idx) = val;
  }

  void SetRange(size_t idx, Dp::Math::real min, Dp::Math::real max) {
    min_val_(idx) = min;
    max_val_(idx) = max;
  }

  //virtual errno_t ApplyLocalCoords() = 0;

  size_t GetNumOfActuators() {
    return actuators_.size();
  }

  std::list<std::shared_ptr<Actuator>>& Actuators() {
    return actuators_;
  }

  virtual Eigen::Matrix3d& Rot() {
    return rot_;
  }

  virtual Eigen::Vector3d& Pos() {
    return pos_;
  }

  const Eigen::Vector6d& GetValue() { return val_;};
  const Eigen::Vector6d& GetVeloc() { return vel_;};
  const Eigen::Vector6d& GetAccel() { return acc_;};
  const Eigen::Vector6d& GetExtFrc() { return efrc_;};
  const Eigen::Vector6d& GetActFrc() { return afrc_;};

  void SetValue (Eigen::Vector6d& value) {val_ = value;};
  void SetVeloc (Eigen::Vector6d& veloc) {vel_ = veloc;};
  void SetAccel (Eigen::Vector6d& accel) {acc_ = accel;};
  void SetExtFrc (Eigen::Vector6d& extfrc) {efrc_ = extfrc;};
  void SetActFrc (Eigen::Vector6d& actfrc) {afrc_ = actfrc;};

  // TODO:
  void SetValue (Dp::Math::real value) {val_(0) = value;};

  std::string& GetName() { return name_;};
};

class RotaryJoint : public Joint {
private:
  Eigen::Vector3d axis_;
  Dp::Math::real& angle_;

public:
  RotaryJoint(const char* name, Eigen::Vector3d axis) :
    Joint(name),
    axis_(axis), 
    angle_(Joint::val_(0))
  {
    actuators_.push_back(
      std::make_shared<Actuator>(val_(0), vel_(0), acc_(0), efrc_(0), afrc_(0))
    );
  };

  virtual ~RotaryJoint() {};

  static std::shared_ptr<RotaryJoint> Create (const char* name, Eigen::Vector3d axis) {
    return std::make_shared<RotaryJoint>(name, axis);
  }

  Eigen::Matrix3d& Rot() {
    rot_ = Eigen::AngleAxisd(angle_, axis_);
    return rot_;
  }

  //errno_t ApplyLocalCoords() {
  //  CasCoords::LRot() = rot();
  //  return 0;
  //}

  errno_t SetAngle (Dp::Math::real angle) {
    angle_ = angle;
    return 0; 
  }
};

using namespace std;
using namespace Eigen;
using namespace Dp::Math;

class Link : public CasCoords {
private:
  //const char* name_;
  std::string name_;

  std::shared_ptr<Joint> joint_;
  std::list<std::shared_ptr<Link>> clinks_;

  Eigen::Vector3d l_tippos_;
  Eigen::Matrix3d l_tiprot_;
  Dp::Math::real     mass_;
  Eigen::Vector3d centroid_; /* center of gravity */
  Eigen::Matrix3d cinertia_; /* inertia tensor at centroid */

  std::shared_ptr<Joint>& getJoint() {
    return joint_;
  }

public:
  //Link(const char* name) : name_(name) {};
  //Link(const char* name, std::shared_ptr<Joint> joint) : name_(name), joint_(joint) {};
  Link(
    const char* name, shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) :
    name_(name), joint_(joint), l_tippos_(lpos), l_tiprot_(Matrix3d::Identity()), mass_(mass), centroid_(centroid),
    cinertia_(cinertia) {};
    //cinertia_(Matrix3d::Identity()) {};
  virtual ~Link() {};

  void SetMass(Dp::Math::real mass) {
    mass_ = mass;
  }

  void SetCentroid(Vector3d centroid) {
    centroid_ = centroid;
  }

  void SetInertia(Matrix3d inertia) {
    cinertia_ = inertia;
  }

  static std::shared_ptr<Link> Create (
    const char* name, shared_ptr<Joint> joint, Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
    Matrix3d cinertia) {
    return std::make_shared<Link>(name, joint, lpos, mass, centroid, cinertia);
  }

  //std::shared_ptr<Joint> GetJoint() {
  //  return joint_;
  //}
  Joint& GetJoint() {
    return *(joint_);
  }

  std::shared_ptr<Joint> FindJoint(std::string& str) {
    /* TODO: duplicate */
    if (joint_->GetName() == str) {
      return joint_;
    }
    for (auto &link : clinks_) {
      auto joint = link->FindJoint(str);
      if (joint != NULL) {
        return joint;
      }
    }
    return NULL;
  }
  std::shared_ptr<Joint> FindJoint(const char* name) {
    std::string tname = name;
    return FindJoint(tname);
  }

  // TODO: --> CasCoords 
  Eigen::Vector3d& LTipPos() {
    return l_tippos_;
  }
  Eigen::Matrix3d& LTipRot() {
    return l_tiprot_;
  }

  errno_t AssignJoint (std::shared_ptr<Joint> joint) {
    joint_ = std::move(joint);
  }

  errno_t AddChild (std::shared_ptr<Link> clink) {
    CasCoords::AddChild(clink);
    clinks_.push_back(clink);
  }

  errno_t ApplyLocalCoords() {
    //CasCoords::LRot() = joint_->Rot() * l_tiprot_;
    CasCoords::LRot() = l_tiprot_ * joint_->Rot();
    CasCoords::LPos() = joint_->Pos() + l_tippos_;
    //std::cout << name_ << std::endl;
    //std::cout << "  ROT:" << CasCoords::LRot() << std::endl;
    //std::cout << "  POS:" << l_tippos_ << std::endl;
    //std::cout << "  POS:" << CasCoords::LPos() << std::endl;
    return 0;
  }

  errno_t UpdateLocals() {
    ECALL(ApplyLocalCoords());
    for (auto &link : clinks_) {
      link->UpdateLocals();
    }
    return 0;
  }

  errno_t UpdateCasCoords() {
    ECALL(UpdateLocals());      /* ローカル位置・姿勢更新 */
    ECALL(CasCoords::Update()); /* ワールド位置・姿勢更新 */
    return 0;
  }

  std::string& GetName() {
    return name_;
  }
};



/* ssg : Simple Scene Graph */
namespace ssg {


  class DrawableLink : public Link {
  private:
    //const char* name_;
    //std::string name_;

    //std::shared_ptr<InterfaceSceneObject> obj_ = NULL;
    std::list<std::shared_ptr<InterfaceSceneObject>> objs_;
  
  public:
  
    DrawableLink(const char* name,
         std::shared_ptr<Joint> joint,
         Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
         Matrix3d cinertia) :
         Link(name, joint, lpos, mass, centroid, cinertia) {
    };
    DrawableLink(const char* name,
         std::shared_ptr<InterfaceSceneObject> obj,
         std::shared_ptr<Joint> joint,
         Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
         Matrix3d cinertia) :
         Link(name, joint, lpos, mass, centroid, cinertia) {
        objs_.push_back(obj);
    };
    DrawableLink(const char* name,
         std::list<std::shared_ptr<InterfaceSceneObject>> objs,
         std::shared_ptr<Joint> joint,
         Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
         Matrix3d cinertia) :
         Link(name, joint, lpos, mass, centroid, cinertia) {
        objs_ = objs;
    };
    virtual ~DrawableLink() {};

    static std::shared_ptr<DrawableLink> Create(const char* name,
         std::shared_ptr<Joint> joint,
         Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
         Matrix3d cinertia) {
      return std::make_shared<DrawableLink>(name, joint, lpos, mass, centroid, cinertia);
    }
    static std::shared_ptr<DrawableLink> Create(const char* name,
         std::shared_ptr<InterfaceSceneObject> obj,
         std::shared_ptr<Joint> joint,
         Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
         Matrix3d cinertia) {
      return std::make_shared<DrawableLink>(name, obj, joint, lpos, mass, centroid, cinertia);
    }
    static std::shared_ptr<DrawableLink> Create(const char* name,
         std::list<std::shared_ptr<InterfaceSceneObject>> objs,
         std::shared_ptr<Joint> joint,
         Vector3d lpos, Dp::Math::real mass, Vector3d centroid,
         Matrix3d cinertia) {
      return std::make_shared<DrawableLink>(name, objs, joint, lpos, mass, centroid, cinertia);
    }

    void AddShape (std::shared_ptr<InterfaceSceneObject> obj) {
      objs_.push_back(obj);
    }

    void AddShape (std::list<std::shared_ptr<InterfaceSceneObject>> objs) {
      objs_.splice(objs_.end(), objs);
    }

    void SetTransformMatrixLocId (GLint id) {
      for (auto &obj: objs_) {
        obj->SetTransformMatrixLocId(id);
      }
    }

  public:
    errno_t Exec(void) {
      for (auto &obj : objs_) {
        obj->Draw(CasCoords::WRot(), CasCoords::WPos());
      }
      return 0;
    }
  };
}

namespace ssg {
  static const Eigen::Vector3d& parseRotaryAxis (std::string &axis_type) {
      static const std::unordered_map<std::string, Eigen::Vector3d> cases = {
        {"Base",        (Eigen::Vector3d){0.0,0.0,0.0}},
        {"RotaryLinkX", (Eigen::Vector3d){1.0,0.0,0.0}},
        {"RotaryLinkY", (Eigen::Vector3d){0.0,1.0,0.0}},
        {"RotaryLinkZ", (Eigen::Vector3d){0.0,0.0,1.0}}
      };
      auto result = cases.find(axis_type);
      return result != cases.end() ? result->second : cases.begin()->second ;
  }

  static errno_t parseLinkInfo (std::ifstream &ifs, Link &link) {
      Dp::Math::real mass;
      ifs >> mass;
      link.SetMass(mass);

      Vector3d centroid;
      ifs >> centroid(0);
      ifs >> centroid(1);
      ifs >> centroid(2);
      //centroid(0) << ifs;
      //centroid(1) << ifs;
      //centroid(2) << ifs;
      link.SetCentroid(centroid);

      Matrix3d inertia;
      ifs >> inertia(0,0);
      ifs >> inertia(1,1);
      ifs >> inertia(2,2);
      ifs >> inertia(1,0);
      ifs >> inertia(2,1);
      ifs >> inertia(2,0);
      inertia(0,1) = inertia(1,0);
      inertia(1,2) = inertia(2,1);
      inertia(0,2) = inertia(2,0);
      //inertia(0, 0) << ifs;
      //inertia(1, 1) << ifs;
      //inertia(2, 2) << ifs;
      //inertia(0, 1) = inertia(1, 0) << ifs;
      //inertia(1, 2) = inertia(2, 1) << ifs;
      //inertia(0, 2) = inertia(2, 0) << ifs;
      link.SetInertia(inertia);

      std::cout << "mass\n";
      std::cout << mass << std::endl;
      std::cout << "centroid\n";
      std::cout << centroid << std::endl;
      std::cout << "inertia\n";
      std::cout << inertia << std::endl;
      std::cout << "---\n";

      return 0;
  }

  static errno_t parseJointInertia (std::ifstream &ifs, Joint &joint) {
      size_t idx;
      Dp::Math::real val;
      ifs >> idx;
      ifs >> val;

      joint.SetInertia(idx, val);
      return 0;
  }
  static errno_t parseJointInertia (std::ifstream &ifs, Link &link) {
      return parseJointInertia(ifs, link.GetJoint());
  }

  static errno_t parseJointViscosity (std::ifstream &ifs, Joint &joint) {
      size_t idx;
      Dp::Math::real val;
      ifs >> idx;
      ifs >> val;

      joint.SetViscosity(idx, val);
      return 0;
  }
  static errno_t parseJointViscosity (std::ifstream &ifs, Link &link) {
      return parseJointViscosity(ifs, link.GetJoint());
  }

  static errno_t parseJointRange (std::ifstream &ifs, Joint &joint) {
      size_t idx;
      Dp::Math::real minval, maxval;
      ifs >> idx;
      ifs >> minval;
      ifs >> maxval;

      joint.SetRange(idx, minval, maxval);
      return 0;
  }
  static errno_t parseJointRange (std::ifstream &ifs, Link &link) {
      return parseJointRange(ifs, link.GetJoint());
  }

  std::list<std::shared_ptr<InterfaceSceneObject>> ImportShapeFile (std::string &filepath);

  static errno_t parseShape (std::ifstream &ifs, Link &link) {

      std::string file;
      ifs >> file;

      /* TODO dynamic_cast to be removed */
      auto shapes = ImportShapeFile(file);
      dynamic_cast<DrawableLink&>(link).AddShape(shapes);

      std::cout << "Shape : " << file << "\n";

      return 0;
  }

  static errno_t parseHull (std::ifstream &ifs, Link &link) {

      std::string file;
      ifs >> file;

      std::cout << "Hull not implemented : " << file << "\n";

      return 0;
  }

  std::shared_ptr<DrawableLink> ImportLinkFile (std::string &filepath);

  static errno_t parseChild (std::ifstream &ifs, Link &link) {

      std::string file;
      ifs >> file;

      Eigen::Vector3d xyz, rpy;
      ifs >> xyz(0);
      ifs >> xyz(1);
      ifs >> xyz(2);
      ifs >> rpy(0);
      ifs >> rpy(1);
      ifs >> rpy(2);
      
      std::cout << "Child : " << file
                << " xyz = " << xyz(0)  << "," << xyz(1) << "," << xyz(2)
                << " rpy = " << rpy(0)  << "," << rpy(1) << "," << rpy(2) << "\n\n\n";

      /* TODO: LTipRot, TipPos */
      auto clink = ImportLinkFile(file);
      clink->LTipPos() = xyz;
      clink->LTipRot() = Dp::Math::rpy2mat3(rpy);
      link.AddChild(clink);

      return 0;
  }

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

  //static Eigen::Matrix3d rpy2mat3 (Eigen::Vector3d rpy) {
  //
  //  //Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
  //  //Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
  //  //Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
  //  //
  //  //Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
  //  //
  //  //Eigen::Matrix3d rotationMatrix = q.matrix();
  //  
  //  auto rangle = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
  //  auto pangle = Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY());
  //  auto yangle = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
  //  auto rpyangle = rangle * pangle * yangle;

  //  return rpyangle.toRotationMatrix();
  //}

  static std::list<std::shared_ptr<InterfaceSceneObject>> parseCompound (std::ifstream &ifs) {
    std::cout << "  : Shape Compound\n";

    std::string str;
    ifs >> str;

    std::list<std::shared_ptr<InterfaceSceneObject>> shapes;

    /* statement guard */
    if (str != "{") {
      fprintf(stderr, "%s : statement error\n", __FUNCTION__);
      return shapes;
    }

    while(1) {
      ifs >> str;
      /* statement guard */
      if (str == "}") {
        break;
      }

      /* YRP or RPY */
      Dp::Math::real deg;
      Eigen::Vector3d xyz, rpy;
      ifs >> xyz(0);
      ifs >> xyz(1);
      ifs >> xyz(2);
      ifs >> deg; rpy(0) = Dp::Math::deg2rad(deg);
      ifs >> deg; rpy(1) = Dp::Math::deg2rad(deg);
      ifs >> deg; rpy(2) = Dp::Math::deg2rad(deg);

      //auto rot = rpy2mat3(rpy) * AngleAxisd(Dp::Math::deg2rad(90), Vector3d::UnitX());
      auto rot = rpy2mat3(rpy);
      auto shape = ImportShapeFile(str);
      for (auto shp: shape) {
        shp->SetOffset(xyz, rot);
      }

      //shapes.push_back(shape);
      shapes.splice(shapes.end(), shape);
    }

    //link.AddShape(shapes);
    return shapes;
  }

  static std::list<std::shared_ptr<InterfaceSceneObject>> parseCylinder (std::ifstream &ifs) {
    std::cout << "  : Shape Cylinder\n";

    Dp::Math::real radius, height;
    ifs >> radius;
    ifs >> height;

    std::list<std::shared_ptr<InterfaceSceneObject>> shapes;

    auto shape = std::make_shared<WiredCylinder>(
      //Eigen::AngleAxisf(Dp::Math::deg2rad( 0), (Vector3f){0,0,1}).toRotationMatrix(),  Eigen::Vector3f::UnitZ()*height/2.0, radius, height, 20);
      Eigen::AngleAxisf(Dp::Math::deg2rad(90), (Vector3f){1,0,0}).toRotationMatrix(),  Eigen::Vector3f::Zero(), radius, height, 20);

    shapes.push_back(shape);

    return shapes;
  }

  static std::list<std::shared_ptr<InterfaceSceneObject>> parseRectangular (std::ifstream &ifs) {
    std::cout << "  : Shape Box\n";

    Dp::Math::real lx,ly,lz;
    ifs >> lx;
    ifs >> ly;
    ifs >> lz;

    std::list<std::shared_ptr<InterfaceSceneObject>> shapes;

    auto shape = std::make_shared<WiredRectangular>(Eigen::Vector3f::Zero(), lx, ly, lz);

    shapes.push_back(shape);

    return shapes;
  }

  static std::list<std::shared_ptr<InterfaceSceneObject>> parseShape (std::string &attr_type, std::ifstream &ifs) {
      static const std::unordered_map<std::string, std::function<std::list<std::shared_ptr<InterfaceSceneObject>>(std::ifstream&)>> cases = {
        {"Cylinder"      , [](std::ifstream &ifs){return parseCylinder(ifs);         }},
        {"Rectangular"   , [](std::ifstream &ifs){return parseRectangular(ifs);      }},
        {"Box"           , [](std::ifstream &ifs){return parseRectangular(ifs);      }},
        {"Compound"      , [](std::ifstream &ifs){return parseCompound(ifs);         }}
      };
      //static const std::unordered_map<std::string, std::function<errno_t(std::ifstream&, Link&)>> cases = {
      //  {"JointInertia"  , parseJointInertia  },
      //  {"JointViscosity", parseJointViscosity},
      //  {"JointRange"    , parseJointRange    }
      //};
      auto result = cases.find(attr_type);
      /* TODO */
      std::list<std::shared_ptr<InterfaceSceneObject>> def;
      return result != cases.end() ? result->second(ifs) : def ;
  }

  std::list<std::shared_ptr<InterfaceSceneObject>> ImportShapeFile (std::string &filepath) {

    std::list<std::shared_ptr<InterfaceSceneObject>> shapes;

    std::ifstream ifs(filepath);
    if (!ifs.good()) {
      std::cout << "error link:" << filepath << "\n";
      return shapes;
    }

    std::string str;
    ifs >> str;

    shapes = parseShape(str, ifs);

    return shapes;
  }

  std::shared_ptr<DrawableLink> ImportLinkFile (std::string &filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.good()) {
      std::cout << "error link:" << filepath << "\n";
      return NULL;
    }

    std::string type;
    std::string name;
    //std::getline(ifs, type, ' ');
    //std::getline(ifs, name, ' ');
    ifs >> type;
    ifs >> name;
    std::cout << "type: " << type << ", name: " << name << "|" << std::endl;

    auto joint = RotaryJoint::Create(name.c_str(), parseRotaryAxis(type));

    auto link = ssg::DrawableLink::Create(name.c_str(), joint, (Vector3d){0.0, 0.0, 0.0}, 0, (Vector3d){0.0,0.0,0.0}, Matrix3d::Zero());

    while(1)
    {
      std::string str;
      ifs >> str;
      std::cout << "===: " << str << std::endl;
      if (ifs.eof()) break;

      std::cout << "--: " << filepath << "  ---:" << str << ":" << ifs.eof() << ":" << std::endl;
      parseLinkAttribute(str, ifs, *link);
    }

    return link;
  }

  std::shared_ptr<DrawableLink> ImportLinkFile (std::string &dirpath, std::string &filename) {
    std::string filepath = dirpath + filename;
    return ImportLinkFile(filepath);
  }

  /* TODO: DrawableLink --> Link */
  std::shared_ptr<DrawableLink> test2(std::string &filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.good()) {
      std::cout << "error link:" << filepath << "\n";
      return NULL;
    }

    /* object name */
    std::string obj_name;
    std::string root_file;
    std::getline(ifs, obj_name);
    std::getline(ifs, root_file);
    /*               file.name --> "" */
    /*              /file.name --> "/" */
    /* hoge/fuga/aho/file.name --> "hoge/fuga/aho/" */
    //std::string data_dir = root_file.substr(0, root_file.find_last_of("/") + 1);
    auto link = ImportLinkFile(root_file);
    if (link == NULL) {
      return NULL;
    }

    std::string str;
    while(std::getline(ifs, str))
    {
      std::string tmp;
      std::istringstream stream(str);
      while(std::getline(stream, tmp, ' '))
      {
        if (tmp == "InitJointValue") {
          std::string link_name;
          std::getline(stream, link_name, ' ');
          Dp::Math::real link_val;
          stream >> link_val;
          auto joint = link->FindJoint(link_name);
          if (joint) {
            joint->SetValue(link_val);
          }
          //node2->GetJoint().SetValue(-rad + Dp::Math::deg2rad(-120));
          /* TODO: SET */
          //std::cout << link_name << ":" << link_val << std::endl;
        }
      }
    }

    return link;
  }

}

Object createShpere (GLfloat radius, GLint pn_r, GLint pn_h) {
  GLfloat r = radius;
  GLfloat top_pos[pn_r + 2][3];

  top_pos[0][0] = 0.0;
  top_pos[0][1] = 0.0;
  top_pos[0][2] =   r;

  for (size_t i = 1; i < pn_r + 2; i++) {
    GLfloat rad = Dp::Math::PI * 1 / pn_h;
    GLfloat r_h   = r * sin(rad);
    GLfloat rad_h = 2 * Dp::Math::PI * i / pn_r;

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
  cameraMatrix(90.0f, 1.0f, 0.05f, 2.0f, temp1);

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
  const GLuint program = loadProgram("point.vert", {"pv", "normal"}, "point.frag", "fc");
  //const GLuint program = loadProgram("point.vert", {"pv"}, "point.frag", "fc");

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
  //sphere obj(0.5, 20, 20);
  //SolidSphere obj(0.5, 20, 3);
  //WiredSphere obj(0.5, 20, 10);
  //SolidCylinder obj(0.5, 1.0, 20);

  //auto obj1  = std::make_shared<WiredCylinder>(                                                             Eigen::Vector3f::Zero()      , 0.50, 0.2, 20);
  //auto obj2  = std::make_shared<WiredCylinder>(                                                             Eigen::Vector3f::UnitZ()*0.25, 0.05, 0.5, 20);
  //auto obj1  = std::make_shared<WiredCylinder>(Eigen::AngleAxisf(Dp::Math::deg2rad( 0), (Vector3f){0,0,1}).toRotationMatrix(), Eigen::Vector3f::Zero()      , 0.05, 0.02, 20);
  //auto obj1  = std::make_shared<WiredCone>( Eigen::Vector3f::Zero(), 0.05, 0.05, 20);
  auto obj1  = std::make_shared<WiredRectangular>(Eigen::Vector3f::Zero(), 1, 1, 0.02);
  Vector3d pos_ = (Vector3d){0.0,0.0,-0.101};
  Matrix3d rot_ = Eigen::Matrix3d::Identity();
  obj1->SetOffset(pos_, rot_);
  auto obj21 = std::make_shared<WiredCylinder>(Eigen::AngleAxisf(Dp::Math::deg2rad(90), (Vector3f){0,1,0}).toRotationMatrix(), Eigen::Vector3f::Zero()      , 0.02, 0.02, 6);
  auto obj22 = std::make_shared<WiredCylinder>(Eigen::AngleAxisf(Dp::Math::deg2rad( 0), (Vector3f){0,0,1}).toRotationMatrix(),  Eigen::Vector3f::UnitZ()*0.025, 0.005, 0.05, 20);
  auto obj31 = std::make_shared<WiredSphere>(0.020, 20, 20);
  auto obj32 = std::make_shared<WiredCylinder>(Eigen::AngleAxisf(Dp::Math::deg2rad( 0), (Vector3f){0,0,1}).toRotationMatrix(),  Eigen::Vector3f::UnitZ()*0.025, 0.005, 0.05, 20);
  auto obj4  = ssg::ImportObject("./test.stl");
  //auto obj4  = ssg::ImportObject("./phoenix_ugv.md2");
  //auto obj4  = std::make_shared<WiredCylinder>(Eigen::AngleAxisf(Dp::Math::deg2rad( 0), (Vector3f){0,0,1}).toRotationMatrix(), Eigen::Vector3f::Zero()      , 0.50, 0.2, 20);
  //auto obj31 = std::make_shared<WiredCylinder>(                                                             Eigen::Vector3f::Zero()      , 0.20, 0.2, 20);
  //auto obj32 = std::make_shared<WiredCylinder>(                                                             Eigen::Vector3f::UnitZ()*0.25, 0.05, 0.5, 20);
  obj1->SetTransformMatrixLocId(transformMatrixLocation);
  obj21->SetTransformMatrixLocId(transformMatrixLocation);
  obj22->SetTransformMatrixLocId(transformMatrixLocation);
  obj31->SetTransformMatrixLocId(transformMatrixLocation);
  obj32->SetTransformMatrixLocId(transformMatrixLocation);
  obj4->SetTransformMatrixLocId(transformMatrixLocation);
  std::list<std::shared_ptr<InterfaceSceneObject>> objs3;
  objs3.push_back(obj31);
  objs3.push_back(obj32);
  std::list<std::shared_ptr<InterfaceSceneObject>> objs2;
  objs2.push_back(obj21);
  objs2.push_back(obj22);
  auto node1 = ssg::DrawableLink::Create("A", obj1,  RotaryJoint::Create("01", Vector3d::UnitX()), (Vector3d){0,0.000,0.00}, 1, (Vector3d){0,0,0}, Matrix3d::Identity());
  auto node2 = ssg::DrawableLink::Create("B", objs2, RotaryJoint::Create("01", Vector3d::UnitX()), (Vector3d){0,0.050,0.00}, 1, (Vector3d){0,0,0}, Matrix3d::Identity());
  auto node3 = ssg::DrawableLink::Create("C", objs3, RotaryJoint::Create("01", Vector3d::UnitX()), (Vector3d){0,0.000,0.05}, 1, (Vector3d){0,0,0}, Matrix3d::Identity());
  auto node4 = ssg::DrawableLink::Create("D", obj4,  RotaryJoint::Create("01", Vector3d::UnitX()), (Vector3d){0,0.000,0.05}, 1, (Vector3d){0,0,0}, Matrix3d::Identity());
  //node1->LPos() = Eigen::Vector3d(0.0,0.0,0.0);
  //node2->LPos() = Eigen::Vector3d(0.0,0.0,0.10);
  //node3->LPos() = Eigen::Vector3d(0.0,0.0,0.50);
  node3->AddChild(node4);
  node2->AddChild(node3);
  node1->AddChild(node2);

  auto rot = Eigen::Matrix3d();
  rot << 1,0,0,
         0,0.9800665778412416, -0.19866933079506122,
         0,0.19866933079506122, 0.9800665778412416;
  node2->LRot() *= rot;
  node3->LRot() *= rot;

  std::string name = "./obj/eV/eV.obj";
  auto node_1 = ssg::test2(name);
  if (node_1 == NULL) {
    fprintf(stderr, "fail to load %s.\n", name.c_str());
    return 1;
  }
  node_1->SetTransformMatrixLocId(transformMatrixLocation);

  GLfloat veloc = 0.005;
  GLfloat cpos[3] = {+0.20, 0.0, 0.0};
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
    //glUniformMatrix4fv(transformMatrixLocation, 1, GL_FALSE, transformMatrix);

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

    auto rot = Eigen::Matrix3d();
    rot << 1,0,0,
           0,0.9800665778412416, -0.19866933079506122,
           0,0.19866933079506122, 0.9800665778412416;
    //node.WPos() += Eigen::Vector3d(0,0,0.01);
    //node3->LRot() *= rot;
    static Dp::Math::real rad = 0.00;
    static bool is_increase = false;
    if (is_increase) {
      rad += 0.02;
      if (rad > Dp::Math::deg2rad(30)) {
        is_increase = false;
      }
    } else {
      rad -= 0.02;
      if (rad < Dp::Math::deg2rad(-30)) {
        is_increase = true;
      }
    }
    node2->GetJoint().SetValue(-rad + Dp::Math::deg2rad(-120));
    node3->GetJoint().SetValue(rad + Dp::Math::deg2rad(-60));
    //obj1->SetScale(1.0 + 10 * Dp::Math::rad2deg(rad) / 30);
    //obj1->SetScale(10 * rad / Dp::Math::deg2rad(60));
    node1->UpdateCasCoords();
    node1->ExecAll();


    node_1->UpdateCasCoords();
    node_1->ExecAll();
    //node_1->FindJoint("FR_HIP_PITCH")->SetValue(-rad + Dp::Math::deg2rad( 0));
    //node_1->FindJoint("FR_KNEE_PITCH")->SetValue( rad + Dp::Math::deg2rad(90));

    //カラーバッファを入れ替える
    glfwSwapBuffers(window);
    //glfwSwapInterval(1);

    //イベントを取り出す
#ifdef __APPLE__
    /* non block */
    glfwPollEvents();
#else
    /* non block */
    //glfwWaitEvents();
    glfwPollEvents();
#endif

    cmeasure.update();

    count++;
  }

  //ssg::test();
  return 0;
}

