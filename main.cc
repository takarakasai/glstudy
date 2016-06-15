#include <stdlib.h>
#include <stdio.h> //#include <cstdlib>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <time.h> // for clock_gettime()
#include <cstring>

#include "cycle_measure.h"

#include <string>

#include "dp_type.h"

#include "Link.h"
#include "Shader.h"

//#include "DrawableLink.h"
#include "ObjFileReader.h"
#include "Camera.h"

#define DPRINTF(...) 

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

#if 0
[[maybe_unused]] static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}
#endif

void cb_resize(GLFWwindow *const window, int width, int height)
{
  glViewport(0, 0, width, height);
}

int main()
{
  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
      exit(EXIT_FAILURE);

  // OpenGL Version 3.2 Core Profileを選択する
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  //glfwWindowHint(GLFW_DECORATED ,false);

  GLuint width = 640;
  GLuint height = 480;
  GLFWwindow* window = glfwCreateWindow(width, height, "Simple example", NULL, NULL);
  //GLFWwindow* window2 = glfwCreateWindow(width, height, "Window2", NULL, window);
  //GLFWwindow* window3 = glfwCreateWindow(width, height, "Window3", NULL, NULL);
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

  glfwSetWindowSizeCallback(window, cb_resize);
  cb_resize(window, width, height);
 
  // 背景色を指定する
  glClearColor(0.0f, 0.0f, 0.2f, 0.0f);

  // hidden surface
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);

  //プログラムオブジェクトを作成する
  const GLuint program = ssg::loadProgram(
      "./shader/point.vert", {"pv", "normal"},
      "./shader/point.frag", "fc");

  std::cout << program << ":PROGRAM" << std::endl;

  const GLint projectionMatrixLocation = glGetUniformLocation(program, "projectionMatrix");
  const GLint transformMatrixLocation = glGetUniformLocation(program, "transformMatrix");
  const GLint materialColorLocation = glGetUniformLocation(program, "materialColor");

  std::string name = "./obj/eV/eV.obj";
  auto node_1 = ssg::test2(name);
  if (node_1 == NULL) {
    fprintf(stderr, "fail to load %s.\n", name.c_str());
    return 1;
  }
  node_1->SetTransformMatrixLocId(transformMatrixLocation);
  node_1->SetMaterialColorLocId(materialColorLocation);

  auto field = ssg::ImportObject("obj/field/ring_assy.stl", 0.001);
  //obj1->SetOffset(pos_, rot_);
  field->SetTransformMatrixLocId(transformMatrixLocation);
  field->SetMaterialColorLocId(materialColorLocation);

  ssg::Camera camera(0.2, 0.0, 0.0,/**/ 0.0, 0.0, 0.0,/**/ 0.0, 0.0, 1.0);

  GLfloat veloc = 0.02;

  cycle_measure cmeasure(5);
  cmeasure.set_cout(true);

  //ウィンドウが開いている間繰り返す
  while (glfwWindowShouldClose(window) == GL_FALSE)
  {
    static int count = 0;

    if (glfwGetKey(window, GLFW_KEY_Q)) {
      break;
    }
    if (glfwGetKey(window, GLFW_KEY_PAGE_UP)) {
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
        camera.Rotate(0.0, 0.05, 0.0);
      } else {
        camera.Rotate(0.0, 0.0, 0.05);
      }
    }
    if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN)) {
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
        camera.Rotate(0.0, -0.05, 0.0);
      } else {
        camera.Rotate(0.0, 0.0, -0.05);
      }
    }
    if (glfwGetKey(window, GLFW_KEY_UP)) {
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
      } else {
        camera += veloc;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN)) {
      if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
      } else {
        camera -= veloc;
      }
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT)) {
      camera += camera.LeftDir() * veloc;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT)) {
      camera -= camera.LeftDir() * veloc;
    }
    if (glfwGetKey(window, GLFW_KEY_W)) {
      node_1->SetDrawMode(false);
    }
    if (glfwGetKey(window, GLFW_KEY_S)) {
      node_1->SetDrawMode(true);
    }

    /*
     * OpenGL shall treat matrix as row-first rule
     * Eigen shall treat matrix as row-first rule at default 
     * So you should make transpose matrix
     */
    GLfloat projectionMatrix[16];
    Eigen::Map<Eigen::Matrix4f>(projectionMatrix, 4, 4) = camera.ProjectionMatrix().transpose().cast<GLfloat>();

    glfwMakeContextCurrent(window);

    // clear window
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // start using shader program
    glUseProgram(program);

    glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, projectionMatrix);

    node_1->UpdateCasCoords();
    node_1->ExecAll();

    {
      Vector3d pos_ = (Vector3d){0.0,0.0,-0.200};
      Matrix3d rot_ = AngleAxisd(Dp::Math::deg2rad(90), Eigen::Vector3d::UnitX()).toRotationMatrix();
      field->Draw(rot_, pos_);
    }

    glfwSwapBuffers(window);

    /* Get EVENT */
    /* non block */
    glfwPollEvents();

    cmeasure.update();

    count++;
  }

  return 0;
}

