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

// for sleep
#include "unistd.h"

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
    bool isVisible_ = true;
    int32_t width_;
    int32_t height_;
    GLFWwindow* window_;
    Camera camera_;
  public:

  private:
    //ssg::Camera camera(0.2, 0.0, 0.0,/**/ 0.0, 0.0, 0.0,/**/ 0.0, 0.0, 1.0);
    Window(GLFWwindow *window, int32_t width, int32_t height) : width_(width), height_(height), window_(window), camera_(0.5,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,1.0) {
      glfwSetWindowSizeCallback(window, cb_resize);
      cb_resize(window, (int)width, (int)height);
    }

  public:

    virtual ~Window() {
    }

    GLFWwindow* WindowHandle () {
      return window_;
    }

    void Show () {
      isVisible_ = true;
      glfwShowWindow(window_);
    }
    void Hide () {
      isVisible_ = false;
      glfwHideWindow(window_);
    }
    //void Focus() {
    //  glfwFocusWindow(window_);
    //}

    Camera& GetCamera() {
      return camera_;
    }

    static std::unique_ptr<Window> Create (size_t width, size_t height, std::string name, GLFWwindow *parent) {
      GLFWwindow* window = glfwCreateWindow(width, height, name.c_str(), NULL, parent);
      return std::unique_ptr<Window>(new Window(window, width, height));
    }

    bool IsVisible () {
      return isVisible_;
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

    cycle_measure cmeasure;//(10);
 public:

    Scene() : cmeasure(10) {
      program_ = ssg::loadProgram(
        "./shader/point.vert", {"pv", "normal"},
        "./shader/point.frag", "fc");

      std::cout << program_ << ":PROGRAM" << std::endl;

      projMatLoc_ = glGetUniformLocation(program_, "projectionMatrix");
      trnsMatLoc_ = glGetUniformLocation(program_, "transformMatrix");
      mateMatLoc_ = glGetUniformLocation(program_, "materialColor");
      
      std::cout << "IDS:" << projMatLoc_ << ", " << trnsMatLoc_ << ", " << mateMatLoc_ << std::endl;

      cmeasure.set_cout(true);
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

      static int count = 0;
      if (count++ > 16) {
        count = 0;
      }

      for (auto &win : windows_) {
        /* TODO IsVisible & Hide */
        if (!win->IsVisible()) {
          continue;
        }
        win->SetCurrent();

       /* TODO: */
       glClearColor(0.0f, 0.0f, 0.2f, 0.0f);
       glEnable(GL_DEPTH_TEST);
       glDepthFunc(GL_LEQUAL);

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

        if (count == 0) {
          cmeasure.update();
          win->SwapBuffers();
        }
      }
      
      /* Get EVENT */
      /* non block */
      glfwPollEvents();

      return 0;
    }
  };
}

errno_t handleWindow (ssg::Window &ssgwindow) {
  const GLfloat veloc = 0.002;

  GLFWwindow* window = ssgwindow.WindowHandle();
  ssg::Camera& camera = ssgwindow.GetCamera();

  if (glfwGetKey(window, GLFW_KEY_Q)) {
    return -1;
  }
  if (glfwGetKey(window, GLFW_KEY_PAGE_UP)) {
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
      camera.Rotate(0.0, 0.0025, 0.0);
    } else {
      camera.Rotate(0.0, 0.0, 0.0025);
    }
  }
  if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN)) {
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
      camera.Rotate(0.0, -0.0025, 0.0);
    } else {
      camera.Rotate(0.0, 0.0, -0.0025);
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

  /* 
   * TODO:
   *  currently you should call Hide() API after instantinating Scene object and caling AddWidow.
   *  Shadar setting should be done before calling Hide() ? : TODO:To be checked 
   */
  /* TODO: remove miliseconds */
  usleep(100*1000);
  window2->Hide();
  scene.AddWindow(window1);
  scene.AddWindow(window2);

  std::string name = "./obj/eV/eV.obj";
  auto robot = ssg::test2(name);
  if (robot == NULL) {
    fprintf(stderr, "fail to load %s.\n", name.c_str());
    return 1;
  }

  robot->UpdateCasCoords();
  scene.AddObject(robot);

  //std::shared_ptr<SceneObject> field = ssg::ImportObject("obj/field/ring_assy.stl", 0.001);
  //Vector3d pos_ = (Vector3d){0.0,0.0,-0.200};
  //Matrix3d rot_ = AngleAxisd(Dp::Math::deg2rad(90), Eigen::Vector3d::UnitX()).toRotationMatrix();
  //field->SetOffset(pos_, rot_);
  std::shared_ptr<SceneObject> field = std::make_shared<SolidRectangular>(Eigen::Vector3f{0,0,-0.220}, 1.0, 1.0, 0.05);
  scene.AddObject(field);

  cycle_measure cmeasure(10);
  //cmeasure.set_cout(true);

  /************************************* ODE ****************************************************************/
  constexpr size_t nol = 5;//17;
  std::shared_ptr<WiredSphere> sphere[nol];
  std::string lname[] = {
    "Base",
    "FR_HIP_YAW", "FR_HIP_PITCH", "FR_KNEE_PITCH", "FR_ANKLE_PITCH",
    "FL/HIP_YAW", "FL/HIP_PITCH", "FL/KNEE_PITCH", "FL/ANKLE_PITCH",
    "BR/HIP_YAW", "BR/HIP_PITCH", "BR/KNEE_PITCH", "BR/ANKLE_PITCH",
    "BL/HIP_YAW", "BL/HIP_PITCH", "BL/KNEE_PITCH", "BL/ANKLE_PITCH"
  };

  dReal angle[nol];
  for (size_t j = 0; j < nol; j++) {
    auto rlink = robot->FindLink(lname[j]);
    angle[j] = rlink->GetJoint().GetAngle();
  }
  //-0.0471976, 1, 0, 6.95163e-310, 6.95163e-310
  //-1.0472,1,0,6.92709e-310,6.92709e-310
  std::cout << "angle ====: " << angle[0] << "," << angle[1] << "," << angle[2] << "," << angle[3] << "," << angle[4] << std::endl;

#if 1
  dInitODE();
  dWorld world;
  world.setGravity(0.0, 0.0, -9.8 / 10);
  //world.setGravity(0, 9.9, 0);
  //world.setERP(dReal erp);
  //world.setCFM(dReal cfm);
  
  //dBodyID     link[NUM];  // link[0] is base link
  //dJointID      joint[NUM]
  dBody link[nol];
  dJoint* joint[nol];

  for (size_t i = 0; i < nol; i++) {
    std::cout << "Name: " << lname[i] << std::endl;
    //auto plink = robot->FindLink(lname[i-1]);
    auto rlink = robot->FindLink(lname[i]);

    dMass m;
    m.setZero();
    auto M = rlink->GetMass();
    auto I = rlink->GetIntertia();
    auto C = rlink->GetCentroid();
    if (i == 0) {
      m.setParameters(2.5, 0,0,0, 2.5,2.5,2.5, 0,0,0);     // mass, cx,cy,cz, I11,I22,I33,I12,I13,I23
    } else {
      m.setParameters(M, C(0),C(1),C(2), I(0,0),I(1,1),I(2,2), I(0,1),I(0,2),I(1,2));     // mass, cx,cy,cz, I11,I22,I33,I12,I13,I23
      m.translate(-C(0),-C(1),-C(2));
    }
    link[i].create(world);
    link[i].setMass(&m);
    std::cout << "mass:" << link[i].getMass().mass << ":" << C(0) << "," << C(1) << "," << C(2) << std::endl;
    link[i].setPosition(rlink->GetWCentroid()(0), rlink->GetWCentroid()(1), rlink->GetWCentroid()(2)); // (dReal x, dReal y, dReal z)

    //std::cout << rlink->GetWCentroid() << std::endl;
    //std::cout << rlink->WPos() << std::endl;

    //sphere[i] = std::make_shared<WiredSphere>(rlink->GetWCentroid().cast<float>(), 0.01, 20, 20);
    if (i == 0) {
      sphere[i] = std::make_shared<WiredSphere>(Eigen::Vector3f::Zero(), 0.03, 8, 8);
    } else {
      sphere[i] = std::make_shared<WiredSphere>(Eigen::Vector3f::Zero(), 0.01, 20, 20);
    }
    scene.AddObject(sphere[i]);
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    sphere[i]->SetOffset(rlink->GetWCentroid(), rot);
    auto color = Eigen::Vector4d{0.5,0.0,0.0,1.0};
    sphere[i]->SetColor(color);
    //link[i].setRotation(); // (dMatrix3)
  }

  dFixedJoint* fjoint;
  joint[0] = fjoint = new dFixedJoint(world, 0);
  //joint[0]->attach(link[0], 0);         // body1, body2
  fjoint->set();
  //std::cout << rlink->WPos()(0) << "," << rlink->WPos()(1) << "," << rlink->WPos()(2) << std::endl;
  for (size_t j = 1; j < nol; j++) {
    std::cout << "Name: " << lname[j] << std::endl;
    //auto plink = robot->FindLink(lname[j-1]);
    auto rlink = robot->FindLink(lname[j]);

    dHingeJoint* jnt;
    joint[j] = jnt = new dHingeJoint(world, 0);
    /* 根元リンクはBaseに拘束する */
    if (j == 1 || j == 5 || j == 9 || j == 13) {
      joint[j]->attach(link[0], link[j]);
    } else {
      joint[j]->attach(link[j-1], link[j]);
    }
    jnt->setAnchor(rlink->WPos()(0), rlink->WPos()(1), rlink->WPos()(2));
    //std::cout << rlink->WPos()(0) << "," << rlink->WPos()(1) << "," << rlink->WPos()(2) << std::endl;
    Eigen::Vector3d waxis = rlink->WRot() * rlink->GetJoint().Axis();
    std::cout << "WAXIS:" << waxis(0) << "," << waxis(1) << "," << waxis(2) << std::endl;
    jnt->setAxis(waxis(0), waxis(1), waxis(2));
  }
#endif

  dJointGroup jgrp;
  dSpace* space = new dHashSpace();
  dGeom* geom[nol];
  dGeom* gem_field = new dPlane (*space, 0/*x*/, 0/*y*/, 1/*z*/, -0.220 + 0.05/2.0/*d*/); /* ax + by + cz = d; */
  geom[0] = new dBox (*space, 0.320, 0.220, 0.030); /* lx, ly, lz */
  geom[0]->setBody(link[0].id());
  for (size_t i = 1; i < nol; i++) {
    geom[i] = new dSphere (*space, 0.01);
    geom[i]->setBody(link[i].id());
  }

  typedef struct {
    dWorld *world;
    dJointGroup *jgrp;
    dSpace *space;
    dGeom* field;
  } ColData;

  ColData coldata = {&world, &jgrp, space, gem_field};

  //typedef void(*CB)(void*,dGeomID,dGeomID);
  auto nearCb = static_cast<void(*)(void*data,dGeomID,dGeomID)>([&](void *data, dGeomID o1, dGeomID o2) {
    ColData *coldata = (ColData*)data;

    dGeomID ground = coldata->field->id();
    dWorld *world = coldata->world;
    dJointGroup* jgrp = coldata->jgrp;

    //std::cout << "nearCB: " << o1 << "," << o2 << std::endl;
    static const int N = 4; // 接触点数の上限は4個
  
    dContact contact[N+10];

    int isGround = ((ground == o1) || (ground == o2)); // 衝突する２つのうちどちらかが地面ならisGroundのフラグを立てる
    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact)); // nは衝突点数

    if (isGround) {
      for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactBounce; // 地面の反発係数を設定
        contact[i].surface.bounce = 0.00; // (0.0~1.0)   反発係数は0から1まで
        contact[i].surface.bounce_vel = 0.00; // (0.0以上)   反発に必要な最低速度
  
        // コンタクトジョイント生成                        
        dJointID c = dJointCreateContact(world->id(), jgrp->id(), &contact[i]);
        //dJointID c = dJointCreateContact(0,0, &contact[i]);
        // 接触している２つのgeometryをコンタクトジョイントで拘束
        dJointAttach (c,dGeomGetBody(contact[i].geom.g1),
                                  dGeomGetBody(contact[i].geom.g2));
      }
    }
    return;
  });

  /**********************************************************************************************************/

  usleep(4000*1000);

  //ウィンドウが開いている間繰り返す
  while (glfwWindowShouldClose(scene.RootWindow().WindowHandle()) == GL_FALSE)
  {
    if (handleWindow(scene.RootWindow()) != 0) {
      std::cout << "BREAKED!!!!|" << std::endl;
      break;
    }

    const dReal* pos4 = link[0].getPosition();
    /* ODE vec3 is 4*1 matrix */
    Eigen::Vector3d pos;
    for (int i = 0; i < 3; i++) {
      pos(i) = pos4[i];
    }
    robot->WPos() = pos;
    /* ODE mat3 is 3*4 matrix */
    const dReal* rot34 = link[0].getRotation();
    Eigen::Matrix3d rot;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        rot(i,j) = rot34[i*4 + j];
      }
    }
    robot->WRot() = rot;
    robot->UpdateCasCoords();
    sphere[0]->SetOffset(pos, rot);

    scene.Draw();

    cmeasure.update();

    /*********  simulation side *************************/
#if 1 
    dReal angle[nol];
    //dReal posz[nol];
    for (size_t j = 1; j < nol; j++) {
      angle[j] = ((dHingeJoint*)joint[j])->getAngle();
      //posz[j]  = link[j].getPosition()[2];
      auto rlink = robot->FindLink(lname[j]);
      /* TODO: 
       *  ワールド座標系の関節角度からリンク座標系の関節角度を算出する */
      Eigen::Vector3d axis = (rlink->WRot() * rlink->GetJoint().Axis()) * angle[j];
      auto axis2 = rlink->WRot().transpose() * axis;
      rlink->GetJoint().SetValue(-axis2.dot(rlink->GetJoint().Axis()));

      Eigen::Vector3d pos(link[j].getPosition()[0], link[j].getPosition()[1], link[j].getPosition()[2]);
      Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
      //sphere[j]->SetOffset(rlink->GetWCentroid(), rot);
      sphere[j]->SetOffset(pos, rot);

      //angle[j] = Dp::Math::rad2deg(angle[j]);
    }
    //std::cout << "angle ----: " << angle[0] << "," << angle[1] << "," << angle[2] << "," << angle[3] << "," << angle[4] << std::endl;
    //std::cout << "posz  ----: " << posz[0] << "," << posz[1] << "," << posz[2] << "," << posz[3] << "," << posz[4] << std::endl;
   
    //space->collide(0, std::is_same(decl_type(result_type(static_cast<void(*)(void*, dGeomID, dGeomID)>(nearCb)))));
    space->collide((void*)&coldata, nearCb);
    //world.step(0.0167);
    //world.step(0.001);
    world.step(0.003);
    jgrp.empty();

    //world.setContactSurfaceLayer(0.01);
#endif
  }

  return 0;
}

