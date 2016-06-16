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

#if 0
[[maybe_unused]] static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}
#endif

namespace ssg {
  static void error_callback(int error, const char* description)
  {
    fputs(description, stderr);
  }

  static errno_t InitGlfw () {
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
      exit(EXIT_FAILURE);

    // OpenGL Version 3.2 Core Profileを選択する
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    return 0;
  }

  static errno_t InitGlew () {
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
      // GLEWの初期化に失敗した
      printf("Can't initialize GLEW\n");
      return 1;
    }
    return 0;
  }
}

namespace ssg {
  void cb_resize(GLFWwindow *const window, int width, int height)
  {
    glViewport(0, 0, width, height);
  }

  class Window {
  private:
    int32_t width_;
    int32_t height_;
    GLFWwindow* window_;
    Camera camera_;
  public:

  private:
    //ssg::Camera camera(0.2, 0.0, 0.0,/**/ 0.0, 0.0, 0.0,/**/ 0.0, 0.0, 1.0);
    Window(GLFWwindow *window, int32_t width, int32_t height) : width_(width), height_(height), window_(window), camera_(0.2,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0) {
      glfwSetWindowSizeCallback(window, cb_resize);
      cb_resize(window, (int)width, (int)height);
    }

  public:

    virtual ~Window() {
    }

    GLFWwindow* WindowHandle () {
      return window_;
    }

    Camera& GetCamera() {
      return camera_;
    }

    static std::unique_ptr<Window> Create (size_t width, size_t height, std::string name, GLFWwindow *parent) {
      GLFWwindow* window = glfwCreateWindow(width, height, name.c_str(), NULL, parent);
      return std::unique_ptr<Window>(new Window(window, width, height));
    }

    errno_t SetCurrent() {
      if (window_ == nullptr) {
        return -1;
      }
      glfwMakeContextCurrent(window_);
      return 0;
    }

    errno_t SwapBuffers() {
      if (window_ == nullptr) {
        return -1;
      }
      glfwSwapBuffers(window_);
      return 0;
    }
  };
}

namespace ssg {
  class Scene {
  private:
    std::list<std::unique_ptr<Window>> windows_;
    std::list<std::shared_ptr<InterfaceSceneObject>> objs_;

    GLuint program_;
    GLint projMatLoc_;
    GLint trnsMatLoc_;
    GLint mateMatLoc_;
 public:

    Scene() {
      program_ = ssg::loadProgram(
        "./shader/point.vert", {"pv", "normal"},
        "./shader/point.frag", "fc");

      std::cout << program_ << ":PROGRAM" << std::endl;

      projMatLoc_ = glGetUniformLocation(program_, "projectionMatrix");
      trnsMatLoc_ = glGetUniformLocation(program_, "transformMatrix");
      mateMatLoc_ = glGetUniformLocation(program_, "materialColor");
      
      std::cout << "IDS:" << projMatLoc_ << ", " << trnsMatLoc_ << ", " << mateMatLoc_ << std::endl;

      /* TODO: */
      glClearColor(0.0f, 0.0f, 0.2f, 0.0f);
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LEQUAL);
    }

    virtual ~Scene() {
    }

    Window& RootWindow () {
      /* TODO */
      return *(windows_.begin()->get());
    }

    errno_t AddWindow (std::unique_ptr<Window>& win) {
      windows_.push_back(std::move(win));
      return 0;
    }

    errno_t AddObject (std::shared_ptr<InterfaceSceneObject> obj) {
      obj->SetTransformMatrixLocId(trnsMatLoc_);
      obj->SetMaterialColorLocId(mateMatLoc_);

      objs_.push_back(obj);
      return 0;
    }

    errno_t Draw() {

      GLfloat matrix44[16];
      //GLfloat vector4[4];

      for (auto &win : windows_) {
        win->SetCurrent();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        // start using shader program
        glUseProgram(program_);

        auto camera = win->GetCamera();
        Eigen::Map<Eigen::Matrix4f>(matrix44, 4, 4) = camera.ProjectionMatrix().transpose().cast<GLfloat>();
        glUniformMatrix4fv(projMatLoc_, 1, GL_FALSE, matrix44);

        for (auto &obj : objs_) {
          obj->Draw();
          /* TODO: should call Draw with Location ids */
          // obj.Draw(locations_list);
        }

        win->SwapBuffers();
      }
      
      return 0;
    }
  };
}

errno_t handleWindow (ssg::Window &ssgwindow) {
  const GLfloat veloc = 0.02;

  GLFWwindow* window = ssgwindow.WindowHandle();
  ssg::Camera& camera = ssgwindow.GetCamera();

  if (glfwGetKey(window, GLFW_KEY_Q)) {
    return -1;
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

  //if (glfwGetKey(window, GLFW_KEY_W)) {
  //  node_1->SetDrawMode(false);
  //}
  //if (glfwGetKey(window, GLFW_KEY_S)) {
  //  node_1->SetDrawMode(true);
  //}
  return 0;
}

#include "ode/ode.h"

int main()
{
  ECALL(ssg::InitGlfw());
  
  std::unique_ptr<ssg::Window> window1 = ssg::Window::Create(680, 480, "window1", NULL);
  std::unique_ptr<ssg::Window> window2 = ssg::Window::Create(680, 480, "window2", window1->WindowHandle());
  window1->SetCurrent();

  ECALL(ssg::InitGlew());

  auto camera1 = window1->GetCamera();
  auto camera2 = window2->GetCamera();

  ssg::Scene scene;
  scene.AddWindow(window1);
  scene.AddWindow(window2);

  std::string name = "./obj/eV/eV.obj";
  auto node_1 = ssg::test2(name);
  if (node_1 == NULL) {
    fprintf(stderr, "fail to load %s.\n", name.c_str());
    return 1;
  }
  scene.AddObject(node_1);

  std::shared_ptr<SceneObject> field = ssg::ImportObject("obj/field/ring_assy.stl", 0.001);
  Vector3d pos_ = (Vector3d){0.0,0.0,-0.200};
  Matrix3d rot_ = AngleAxisd(Dp::Math::deg2rad(90), Eigen::Vector3d::UnitX()).toRotationMatrix();
  field->SetOffset(pos_, rot_);
  scene.AddObject(field);

  cycle_measure cmeasure(5);
  cmeasure.set_cout(true);

  //ウィンドウが開いている間繰り返す
  while (glfwWindowShouldClose(scene.RootWindow().WindowHandle()) == GL_FALSE)
  {
    if (handleWindow(scene.RootWindow()) != 0) {
      std::cout << "BREAKED!!!!|" << std::endl;
      break;
    }

    node_1->UpdateCasCoords();

    scene.Draw();

    /* Get EVENT */
    /* non block */
    glfwPollEvents();

    cmeasure.update();
  }

  return 0;
}

