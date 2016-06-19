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

#include <iomanip>

#include "ode/ode.h"
#include "ode/collision_trimesh.h"

namespace ode {
  Eigen::Vector3d vec32vec (const dReal *vec3) {
    /* ODE vec3 is 4*1 matrix */
    Eigen::Vector3d vec;
    for (int i = 0; i < 3; i++) {
      vec(i) = vec3[i];
    }

    return vec;
  }

  Eigen::Matrix3d mat32mat (const dReal *mat3) {
    /* ODE mat3 is 3*4 matrix */
    Eigen::Matrix3d mat;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        mat(i,j) = mat3[i*4 + j];
      }
    }

    return mat;
  }

  Eigen::Vector3d vec2vec3 (const Eigen::Vector3d& vec, dReal *vec3) {
    /* ODE vec3 is 4*1 matrix */
    for (int i = 0; i < 3; i++) {
      vec3[i] = vec(i);
    }
    vec3[3] = 1.0;

    return vec;
  }

  Eigen::Matrix3d mat2mat3 (const Eigen::Matrix3d& mat,  dReal *mat3) {
    /* ODE mat3 is 3*4 matrix */
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        mat3[i*4 + j] = mat(i,j);
      }
      mat3[i*4 + 3] = 0.0;
    }

    return mat;
  }
};

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
  //scene.AddObject(field);

  cycle_measure cmeasure(10);
  //cmeasure.set_cout(true);

  /************************************* ODE ****************************************************************/
  constexpr size_t nol = 17;
  std::shared_ptr<WiredSphere> sphere[nol];
  std::string lname[] = {
    "Base",
    "FR/HIP_YAW", "FR/HIP_PITCH", "FR/KNEE_PITCH", "FR/ANKLE_PITCH",
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
  //dWorldSetContactMaxCorrectingVel(world.id(), 20);
  world.setGravity(0.0, 0.0, -9.8);
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

    if (i == 0) {
      sphere[i] = std::make_shared<WiredSphere>(Eigen::Vector3f::Zero(), 0.03, 8, 8);
    } else if (i == 1 || i == 5 || i == 9 || i == 13) {
      //sphere[i] = std::make_shared<WiredCylinder>(Eigen::, , 0.01/*radius*/, 0.02/*length*/);
      sphere[i] = std::make_shared<WiredSphere>(Eigen::Vector3f::Zero(), 0.01, 20, 20);
    } else {
      //geom[i] = new dCylinder (*space, 0.01/*radius*/, 0.10 * 2.5/*length*/);
      sphere[i] = std::make_shared<WiredSphere>(Eigen::Vector3f::Zero(), 0.01, 20, 20);
    }
    scene.AddObject(sphere[i]);
    sphere[i]->SetOffset(rlink->GetWCentroid(), rlink->WRot());
    auto color = Eigen::Vector4d{0.5,0.0,0.0,1.0};
    sphere[i]->SetColor(color);
    dMatrix3 rot3;
    ode::mat2mat3(rlink->WRot(), rot3);
    link[i].setRotation(rot3);
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

  /* mesh */
  Vector3d _pos = (Vector3d){0.0,0.0,-0.350};
  Matrix3d _rot = AngleAxisd(Dp::Math::deg2rad(90), Eigen::Vector3d::UnitX()).toRotationMatrix();
  std::shared_ptr<ssg::SolidMesh> kawasaki_field = ssg::ImportObject("obj/field/ring_assy.stl", 0.001, _rot, _pos);
  scene.AddObject(kawasaki_field);

  dTriMeshDataID Data = dGeomTriMeshDataCreate();
  ssg::Vertices& verts = kawasaki_field->GetVertices();
  std::vector<GLuint>& idx = kawasaki_field->GetIndices();
  dGeomTriMeshDataBuildSingle1(
            Data,
            &verts.poses[0], 3 * sizeof(float), verts.poses.size(),
            &idx[0], idx.size(), 3*sizeof(GLuint), (void*)&verts.norms[0]);
  dGeomID TriMesh = dCreateTriMesh(space->id()/*spaceID*/, Data, NULL, NULL, NULL);
  dGeomSetData(TriMesh/*geomID*/, Data/*TriMesh*/);

  /* collision */
  dGeom* geom[nol];
  //dGeom* gem_field = new dPlane (*space, 0/*x*/, 0/*y*/, 1/*z*/, -0.220 + 0.05/2.0/*d*/); /* ax + by + cz = d; */
  geom[0] = new dBox (*space, 0.320, 0.220, 0.030); /* lx, ly, lz */
  //geom[0] = new dSphere (*space, 0.03);
  geom[0]->setBody(link[0].id());
  std::cout << "GEOM_ID: " << "K" << " " << TriMesh << std::endl;
  //std::cout << "GEOM_ID: " << "F" << " " << gem_field->id() << std::endl;
  std::cout << "GEOM_ID: " <<  0  << " " << geom[0]->id() << std::endl;
  for (size_t i = 1; i < nol; i++) {
    //geom[i] = new dSphere (*space, 0.01);
    if (i == 1 || i == 5 || i == 9 || i == 13) {
      geom[i] = new dCylinder (*space, 0.01/*radius*/, 0.02/*length*/);
    } else {
      geom[i] = new dCylinder (*space, 0.01/*radius*/, 0.10/*length*/);
    }
    geom[i]->setBody(link[i].id());

    std::cout << "GEOM_ID: " << i << " " << geom[i]->id() << std::endl;
  }

  typedef struct {
    dWorld *world;
    dJointGroup *jgrp;
    dSpace *space;
    dGeom* field;
    dGeomID kawasaki_field;
    ssg::Scene* scene;
  } ColData;

  ColData coldata = {&world, &jgrp, space, NULL/*gem_field*/, TriMesh, &scene};

  //typedef void(*CB)(void*,dGeomID,dGeomID);
  auto nearCb = static_cast<void(*)(void*data,dGeomID,dGeomID)>([&](void *data, dGeomID o1, dGeomID o2) {
    ColData *coldata = (ColData*)data;

    // TODO:
    //dGeomID ground = coldata->field->id();
    dGeomID ground = coldata->kawasaki_field;
    dWorld *world = coldata->world;
    dJointGroup* jgrp = coldata->jgrp;

    //std::cout << "nearCB: " << o1 << "," << o2 << std::endl;
    static const int N = 8; // 接触点数の上限は4個
  
    dContact contact[N+10];

    int isGround = ((ground == o1) || (ground == o2)); // 衝突する２つのうちどちらかが地面ならisGroundのフラグを立てる
    int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact)); // nは衝突点数

    if (isGround) {
      //std::cout << "[" << n << "] : " << o1 << "," << o2 << std::endl;
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

        //std::cout << contact[i].geom.pos[0] << "," << contact[i].geom.pos[1] << "," << contact[i].geom.pos[2] << std::endl;
        Eigen::Vector3f pos;
        pos(0) = contact[i].geom.pos[0];
        pos(1) = contact[i].geom.pos[1];
        pos(2) = contact[i].geom.pos[2];
        //auto point = std::make_shared<WiredSphere>(pos, 0.03, 8, 8);
        //coldata->scene->AddObject(point);
      }
    }
    return;
  });

  /**********************************************************************************************************/

  //usleep(4000*1000);

  //ウィンドウが開いている間繰り返す
  while (glfwWindowShouldClose(scene.RootWindow().WindowHandle()) == GL_FALSE)
  {
    if (handleWindow(scene.RootWindow()) != 0) {
      std::cout << "BREAKED!!!!|" << std::endl;
      break;
    }

    const dReal* pos4 = link[0].getPosition();
    robot->WPos() = ode::vec32vec(pos4);
    const dReal* rot34 = link[0].getRotation();
    robot->WRot() = ode::mat32mat(rot34);
    robot->UpdateCasCoords();

    /* TODO: body centre */
    sphere[0]->SetOffset(robot->WPos(), robot->WRot());

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
   
    space->collide((void*)&coldata, nearCb);
    //world.step(0.0167);
    world.step(0.001);
    //world.step(0.0001);
    jgrp.empty();

#endif
  }

  return 0;
}

