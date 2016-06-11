#include <stdlib.h>
#include <stdio.h> //#include <cstdlib>
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
//#include "VertexObject.h"
//#include "Vertex.h"
//#include "Utils.h"
#include "Mesh.h"
#include "PrimitiveObject.h"

#include "DrawableLink.h"
#include "ObjFileReader.h"

using namespace Eigen;

static GLfloat aspect_ratio = 0;
static GLfloat size[2] = {0,0};
static GLfloat dpm = 100.0;

#define DPRINTF(...) 

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}

#include <math.h>

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
  ssg::cameraMatrix(90.0f, 1.0f, 0.05f, 2.0f, temp1);
  Eigen::Matrix4d camera_mat = ssg::cameraMatrix(90.0, 1.0, 0.05, 2.0);

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
  const GLuint program = ssg::loadProgram("point.vert", {"pv", "normal"}, "point.frag", "fc");
  //const GLuint program = loadProgram("point.vert", {"pv"}, "point.frag", "fc");

  //const GLint aspectLoc = glGetUniformLocation(program, "aspect");
  const GLint projectionMatrixLocation = glGetUniformLocation(program, "projectionMatrix");
  const GLint transformMatrixLocation = glGetUniformLocation(program, "transformMatrix");
  const GLint sizeLoc = glGetUniformLocation(program, "size");
  const GLint dpmLoc = glGetUniformLocation(program, "dpm");
  printf("sizeLoc : %d\n", sizeLoc);
  printf("dpmLoc  : %d\n", dpmLoc);

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
  //auto obj4  = ssg::ImportObject("./obj/stl/test.stl");
  //auto obj4  = ssg::ImportObject("./phoenix_ugv.md2");
  auto obj4  = std::make_shared<WiredCylinder>(Eigen::AngleAxisf(Dp::Math::deg2rad( 0), (Vector3f){0,0,1}).toRotationMatrix(), Eigen::Vector3f::Zero()      , 0.02, 0.005, 20);
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
    Eigen::Vector3d cp = Eigen::Map<Eigen::Vector3f>(cpos, 3).cast<double>();
    Eigen::Vector3d cd = Eigen::Map<Eigen::Vector3f>(cdir_to, 3).cast<double>();
    Eigen::Matrix4d m =  ssg::lookAt(cp, cd);
    std::cout << ssg::lookAt(cp, cd) << std::endl;
    std::cout << "===" << std::endl;

    //Eigen::Matrix4d projection_mat = ssg::lookAt(cpos[0], cpos[1], cpos[2], cdir_to[0], cdir_to[1], cdir_to[2], 0.0f, 0.0f, 1.0f) * camera_mat;
    Eigen::Matrix4d projection_mat = ssg::lookAt(cp, cd) * camera_mat;
    Eigen::Map<Eigen::Matrix4f>(projectionMatrix, 4, 4) = projection_mat.transpose().cast<GLfloat>();

    std::cout << ssg::lookAt(cpos[0], cpos[1], cpos[2], cdir_to[0], cdir_to[1]    , cdir_to[2], 0.0f, 0.0f, 1.0f) << std::endl;
    std::cout << "===========================" << std::endl;

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


    //node_1->FindJoint("FR_HIP_PITCH")->SetValue(-rad + Dp::Math::deg2rad( 0));
    //node_1->FindJoint("FR_KNEE_PITCH")->SetValue( rad + Dp::Math::deg2rad(90));
    node_1->UpdateCasCoords();
    node_1->ExecAll();

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

