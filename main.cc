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

#define WORLD_STEP 0.001

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

    static std::unique_ptr<Window> Create (size_t width, size_t height, std::string name, GLFWmonitor *monitor, GLFWwindow *parent) {
      GLFWwindow* window = glfwCreateWindow(width, height, name.c_str(), monitor, parent);
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
    GLint texLoc_;
    GLint mateMatLoc_;

    cycle_measure cmeasure;//(10);
 public:

    Scene() : cmeasure(10) {
      program_ = ssg::loadProgram(
        "./shader/point.vert", {"pv", "normal", "tex"},
        "./shader/point.frag", "fc");

      if (program_ == -1) {
        exit(1);
      }

      std::cout << program_ << ":PROGRAM" << std::endl;

      projMatLoc_ = glGetUniformLocation(program_, "projectionMatrix");
      trnsMatLoc_ = glGetUniformLocation(program_, "transformMatrix");
      texLoc_     = glGetUniformLocation(program_, "texDiff");
      mateMatLoc_ = glGetUniformLocation(program_, "materialColor");
      
      std::cout << "IDS:" << projMatLoc_ << ", " << trnsMatLoc_ << ", " << mateMatLoc_ << texLoc_ << std::endl;

      //cmeasure.set_cout(true);
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
      obj->SetTextureLocId(texLoc_);

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
       //glDepthFunc(GL_CULL_FACE);

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
    
    double FPS() {
      return cmeasure.FPS();
    }
  };
}

errno_t handleWindow (ssg::Window &ssgwindow) {
  static GLfloat veloc = 0.0020;

  GLFWwindow* window = ssgwindow.WindowHandle();
  ssg::Camera& camera = ssgwindow.GetCamera();

  if (glfwGetKey(window, GLFW_KEY_Q)) {
    return -1;
  }
  if (glfwGetKey(window, GLFW_KEY_P)) {
    if (veloc < 0.0100) {
      veloc += 0.00001;
    }
  } else if (glfwGetKey(window, GLFW_KEY_M)) {
    if (veloc > 0.0002) {
      veloc -= 0.00001;
    }
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
      camera += camera.Top() * veloc * 0.2;
    } else {
      camera += veloc;
    }
  }
  if (glfwGetKey(window, GLFW_KEY_DOWN)) {
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
      camera -= camera.Top() * veloc * 0.2;
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

#include <SDL2/SDL_ttf.h>
//#include <FTGL/FTGLPolygonFont.h>
//#include <FTGL/ftgl.h>

//const char* fontfile = "/usr/share/fonts/truetype/takao-gothic/TakaoGothic.ttf";
#if defined(__APPLE__)
//const char* fontfile = "/System//Library/Fonts/Keyboard.ttf";
const char* fontfile = "/usr//local/texlive/2015/texmf-dist/fonts/truetype/public/gnu-freefont/FreeMonoBold.ttf";
#else
const char* fontfile = "/usr/share/fonts/truetype/freefont/FreeMonoBold.ttf";
#endif


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

#include <random>

int main()
{
  ECALL(ssg::InitGlfw());

  int nom = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&nom);
  for (ssize_t i = 0; i < nom; i++) {
    printf("[%zd] %s\n", i, glfwGetMonitorName(monitors[i]));
  }
  
  std::unique_ptr<ssg::Window> window1 = ssg::Window::Create(680, 480, "window1", NULL, NULL);
  //std::unique_ptr<ssg::Window> window2 = ssg::Window::Create(680, 480, "window2", NULL, window1->WindowHandle());

  if (nom > 1) {
    glfwSetWindowPos(window1->WindowHandle(), 0, 1200);
  }
  /* can use after GLFW3.2 */
  //glfwSetWindowMonitor(window1->WindowHandle(), NULL, 0/*x*/, 0/*y*/, 680/*w*/, 480/*h*/, GLFW_DONT_CARE/*rate*/);

  window1->SetCurrent();

  ECALL(ssg::InitGlew());

  auto camera1 = window1->GetCamera();
  //auto camera2 = window2->GetCamera();

  ssg::Scene scene;

  /* 
   * TODO:
   *  currently you should call Hide() API after instantinating Scene object and caling AddWidow.
   *  Shadar setting should be done before calling Hide() ? : TODO:To be checked 
   */
  /* TODO: remove miliseconds */
  usleep(100*1000);
  //window2->Hide();
  scene.AddWindow(window1);
  //scene.AddWindow(window2);

  std::string name = "./obj/eV/eV.obj";
  auto robot = ssg::test2(name);
  if (robot == NULL) {
    fprintf(stderr, "fail to load %s.\n", name.c_str());
    return 1;
  }
  robot->UpdateCasCoords();
  scene.AddObject(robot);

  std::string khr3_name = "obj/khr3-hv/khr3-hv.obj";
  auto khr3 = ssg::test2(khr3_name);
  if (khr3 == NULL) {
    fprintf(stderr, "fail to load %s.\n", khr3_name.c_str());
    return 1;
  }
  ////khr3->SetOffset(Eigen::Vector3d(0.0,-0.3,0.3), Eigen::Matrix3d::Identity());
  khr3->WPos() = Eigen::Vector3d(0.0,-0.2,0.1);
  khr3->UpdateCasCoords();
  //khr3->SetDrawMode(InterfaceSceneObject::DrawMode::WIRED );
  scene.AddObject(khr3);
 
  //std::shared_ptr<SceneObject> field = ssg::ImportObject("obj/field/ring_assy.stl", 0.001);
  //Vector3d pos_ = (Vector3d){0.0,0.0,-0.200};
  //Matrix3d rot_ = AngleAxisd(Dp::Math::deg2rad(90), Eigen::Vector3d::UnitX()).toRotationMatrix();
  //field->SetOffset(pos_, rot_);
  
  std::shared_ptr<SceneObject> field = std::make_shared<SolidRectangular>(Eigen::Vector3f{0,0,-0.220}, 1.0, 1.0, 0.05);
  //scene.AddObject(field);

  cycle_measure cmeasure(10);
  cmeasure.set_cout(true);

  constexpr size_t nok = 3+(6*2)+(4*2);
  std::shared_ptr<SolidSphere> ksphere[nok];
  std::string khr_lname[] = {
    "Base", "BREST_YAW", "HEAD_YAW",
    "rleg/HIP_YAW", "rleg/HIP_ROLL", "rleg/HIP_PITCH", "rleg/KNEE_PITCH", "rleg/ANKLE_PITCH", "rleg/ANKLE_ROLL",
    "lleg/HIP_YAW", "lleg/HIP_ROLL", "lleg/HIP_PITCH", "lleg/KNEE_PITCH", "lleg/ANKLE_PITCH", "lleg/ANKLE_ROLL",
    "rarm/SHOULDER_PITCH", "rarm/SHOULDER_ROLL", "rarm/ELBOW_YAW", "rarm/LIST_PITCH",
    "larm/SHOULDER_PITCH", "larm/SHOULDER_ROLL", "larm/ELBOW_YAW", "larm/LIST_PITCH"
  };

  for (size_t i = 0; i < nok; i++) {
    std::cout << "Name: " << khr_lname[i] << std::endl;
    //auto plink = robot->FindLink(lname[i-1]);
    auto rlink = khr3->FindLink(khr_lname[i]);
    if (!rlink) continue;

    ksphere[i] = std::make_shared<SolidSphere>(Eigen::Vector3f::Zero(), 0.003, 8, 8);
    scene.AddObject(ksphere[i]);
    ksphere[i]->SetOffset(rlink->GetWCentroid(), rlink->WRot());
    auto color = Eigen::Vector4d{0.5,0.0,0.0,1.0};
    ksphere[i]->SetColor(color);
  }

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

  dInitODE();
  dWorld world;
  dSpace* space = new dHashSpace();
  //dWorldSetContactMaxCorrectingVel(world.id(), 20);
  //world.setGravity(0.0, 0.0, -9.8);
  world.setGravity(0.0, 0.0, -9.8);
  //world.setGravity(0, 9.9, 0);
  //world.setERP(dReal erp);
  //world.setCFM(dReal cfm);
  world.setContactSurfaceLayer(0.08);

  class ODELink {
  private:
    std::shared_ptr<Link> parent_;
    /* should be Link not DrawableLInk */
    std::shared_ptr<ssg::DrawableLink> link_;
    std::list<std::shared_ptr<ODELink>> ode_clinks_;

    dBody body_;
    dMass mass_;
    std::shared_ptr<dJoint> joint_;
    dJointFeedback feedback_;

    /* for collision */
    std::list<dTriMeshDataID> col_datas_;
    std::list<dGeomID> geom_ids_;
    dGeom *test_geom;

  public:
    ODELink() {
    }
    virtual ~ODELink() {
    }

    void SetLink (std::shared_ptr<ssg::DrawableLink> link) {
      link_ = link;
    }

    std::shared_ptr<dJoint> FindJoint(std::string& jname) {
      if (link_->GetName() == jname) {
        return joint_;
      }

      for (auto &ode_clink : ode_clinks_) {
        auto joint = ode_clink->FindJoint(jname);
        if (joint) {
          return joint;
        }
      }

      return nullptr;
    }

  private:
    /* TODO: world */
    errno_t configLink (dWorld &world) {
      mass_.setZero();

      auto M = link_->GetMass();
      auto I = link_->GetIntertia();
      auto C = link_->GetCentroid();
      // setParameters --> mass, cx,cy,cz, I11,I22,I33,I12,I13,I23
      mass_.setParameters(M, C(0),C(1),C(2), I(0,0),I(1,1),I(2,2), I(0,1),I(0,2),I(1,2));
      mass_.translate(-C(0),-C(1),-C(2));

      body_.create(world);
      body_.setMass(&mass_);
      std::cout << " " << link_->GetName() << " mass:" << body_.getMass().mass << ":" << C(0) << "," << C(1) << "," << C(2) << std::endl;
      auto WC = link_->GetWCentroid();
      std::cout << " " << "             :" << WC(0) << "," << WC(1) << "," << WC(2) << std::endl;
      body_.setPosition(WC(0), WC(1), WC(2)); // (dReal x, dReal y, dReal z)
  
      dMatrix3 rot3;
      ode::mat2mat3(link_->WRot(), rot3);
      body_.setRotation(rot3);

      return 0;
    }

    errno_t linkWorld (dWorld &world, ODELink *oparent, std::shared_ptr<Link> parent) {
      ECALL(configLink(world));

      parent_ = parent;
      if (parent_) {
        /* TODO: switch following to any kind of Hinge Joint. */
        auto joint = std::make_shared<dHingeJoint>(world/*, 0 *//* JointGroupID */);
        joint->attach(oparent->body_, body_);

        joint_ = joint;
        auto wpos = link_->WPos();
        joint->setAnchor(wpos(0), wpos(1), wpos(2));
  
        Eigen::Vector3d waxis = link_->WRot() * link_->GetJoint().Axis();
        joint->setAxis(waxis(0), waxis(1), waxis(2));
        //std::cout << "KHR WAXIS:" << waxis(0) << "," << waxis(1) << "," << waxis(2) << std::endl;
 
        std::cout << "MIN:" << link_->GetJoint().GetMinAngle() << std::endl;
        std::cout << "MAX:" << link_->GetJoint().GetMaxAngle() << std::endl;
        joint_->setParam(dParamLoStop, link_->GetJoint().GetMinAngle()); //-Dp::Math::deg2rad(180));
        joint_->setParam(dParamHiStop, link_->GetJoint().GetMaxAngle());//+Dp::Math::deg2rad(180));
  
        joint_->setFeedback(&feedback_);

        joint_ = joint;
      } else {
        /* TODO: 2nd argument */
        auto joint = std::make_shared<dFixedJoint>(world/*, 0 */);
        //joint->attach(body_, 0); /* body1, body2 */
        joint->set();

        joint_ = joint;
      }

      for (auto link : link_->GetChilds()) {
        auto ode_clink = std::make_shared<ODELink>();

        std::shared_ptr<ssg::DrawableLink> clink = std::dynamic_pointer_cast<ssg::DrawableLink>(link);
        ode_clink->SetLink(clink);
        ode_clinks_.push_back(ode_clink);

        //ECALL(ode_clink->linkWorld(world, link_/*parent*/));
        ECALL(ode_clink->linkWorld(world, this, link_/*parent*/));

        //ode_clink->joint_->attach(body_, ode_clink->body_);

        //std::cout << " " << link_->GetName() << " <-> " << clink->GetName() << std::endl;;
      }

      return 0;
    }

  public:
    errno_t MakeCollider (const dSpace &space) {
      /* TODO: currently only TriMesh collision available */
      auto objs = link_->GetShapes();
      for (auto &obj : objs) {
        if (link_->GetName() !=  "rleg/ANKLE_ROLL" && link_->GetName() !=  "lleg/ANKLE_ROLL") break;
        ssg::Vertices& verts = obj->GetVertices();
        std::vector<GLuint>& idx = obj->GetIndices();
        dTriMeshDataID data = dGeomTriMeshDataCreate();

        std::vector<Eigen::Vector3f> poses; /* position vectors  */
        for (auto &pose : verts.poses) {
          poses.push_back(pose * 0.001);
        }
        dGeomTriMeshDataBuildSingle1(
                data,
                &verts.poses[0], 3 * sizeof(float), verts.poses.size(),
                &idx[0], idx.size(), 3*sizeof(GLuint), (void*)&verts.norms[0]);
        
        dGeomID geom_id = dCreateTriMesh(space.id()/*spaceID*/, data, NULL, NULL, NULL);
        dGeomSetData(geom_id/*geomID*/, data/*TriMesh*/);

        //geom[0]->setBody(link[0].id());
        dGeomSetBody(geom_id, body_);
        std::cout << " GEOMID:" << geom_id << std::endl;
        
        //test_geom = new dSphere (space, 0.001);
        //test_geom = new dBox (space, 0.005, 0.003, 0.030); /* lx, ly, lz */

        //std::cout << " NAME: "  << link_->GetName() << ", " << test_geom->id() << std::endl;

        //test_geom->setBody(body_.id());

        col_datas_.push_back(data);
        geom_ids_.push_back(geom_id);
      }

      for (auto ode_clink : ode_clinks_) {
        ode_clink->MakeCollider(space);
      }

      return 0;
    }

  public:
    errno_t LinkWorld (dWorld &world) {
      return linkWorld(world, NULL, NULL);
    }

    errno_t World2Link (dWorld &world) {
      if (link_ == NULL) return -1;
      /* TODO: when body_ is not created the, exception is happen at body_.getPosition etc.. */

      if (parent_) {
        /* TODO: must remove down cast */
        //auto angle = ((std::shared_ptr<dHingeJoint>)joint_)->getAngle();
        std::shared_ptr<dHingeJoint> jnt = std::dynamic_pointer_cast<dHingeJoint>(joint_);
        auto angle = jnt->getAngle();

        /* TODO: 
         *  ワールド座標系の関節角度からリンク座標系の関節角度を算出する */
        Eigen::Vector3d axis = (link_->WRot() * link_->GetJoint().Axis()) * angle;
        auto axis2 = link_->WRot().transpose() * axis;
        link_->GetJoint().SetValue(-axis2.dot(link_->GetJoint().Axis()));
      } else {
        const dReal* pos4 = body_.getPosition();
        link_->WPos() = ode::vec32vec(pos4);
        const dReal* rot34 = body_.getRotation();
        link_->WRot() = ode::mat32mat(rot34);
      }

      //Eigen::Vector3d pos(body_.getPosition()[0], body_.getPosition()[1], body_.getPosition()[2]);
      //Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
      for (auto ode_clink : ode_clinks_) {
        ode_clink->World2Link(world);
      }
 
      return 0;
    }
  };

  ODELink ode_link;
  ode_link.SetLink(khr3);
  ode_link.LinkWorld(world);
  ode_link.MakeCollider(*space);
  
  //dBodyID     link[NUM];  // link[0] is base link
  //dJointID      joint[NUM]
  dBody link[nol];
  dJoint* joint[nol];

  /*
   * dJointFeedback has f1,t1,f2,t2 of dvector3
   * */
  dJointFeedback jFb[nol];

  for (size_t i = 0; i < nol; i++) {
    std::cout << "Name: " << lname[i] << std::endl;
    auto rlink = robot->FindLink(lname[i]);

    dMass m;
    m.setZero();
    auto M = rlink->GetMass();
    auto I = rlink->GetIntertia();
    auto C = rlink->GetCentroid();
    m.setParameters(M, C(0),C(1),C(2), I(0,0),I(1,1),I(2,2), I(0,1),I(0,2),I(1,2));     // mass, cx,cy,cz, I11,I22,I33,I12,I13,I23
    m.translate(-C(0),-C(1),-C(2));

    link[i].create(world);
    link[i].setMass(&m);
    std::cout << "mass:" << link[i].getMass().mass << ":" << C(0) << "," << C(1) << "," << C(2) << std::endl;
    link[i].setPosition(rlink->GetWCentroid()(0), rlink->GetWCentroid()(1), rlink->GetWCentroid()(2)); // (dReal x, dReal y, dReal z)

    dMatrix3 rot3;
    ode::mat2mat3(rlink->WRot(), rot3);
    link[i].setRotation(rot3);
  }

  dFixedJoint* fjoint;
  joint[0] = fjoint = new dFixedJoint(world, 0);
  //joint[0]->attach(link[0], 0);         // body1, body2
  fjoint->set();
  for (size_t j = 1; j < nol; j++) {
    std::cout << "Name: " << lname[j] << std::endl;
    auto rlink = robot->FindLink(lname[j]);

    dHingeJoint* jnt;
    joint[j] = jnt = new dHingeJoint(world/*, 0 *//* JointGroupID*/);
    /* 根元リンクはBaseに拘束する */
    if (j == 1 || j == 5 || j == 9 || j == 13) {
      joint[j]->attach(link[0], link[j]);
    } else {
      joint[j]->attach(link[j-1], link[j]);
    }
    jnt->setAnchor(rlink->WPos()(0), rlink->WPos()(1), rlink->WPos()(2));

    Eigen::Vector3d waxis = rlink->WRot() * rlink->GetJoint().Axis();
    std::cout << "WAXIS:" << waxis(0) << "," << waxis(1) << "," << waxis(2) << std::endl;
    jnt->setAxis(waxis(0), waxis(1), waxis(2));
    jnt->setParam(dParamLoStop, -Dp::Math::deg2rad(120));
    jnt->setParam(dParamHiStop, +Dp::Math::deg2rad(120));

    jnt->setFeedback(&jFb[j]);
  }

  for (size_t i = 0; i < nol; i++) {
    std::cout << "Name: " << lname[i] << std::endl;
    auto rlink = robot->FindLink(lname[i]);

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

  }

  dJointGroup jgrp;

  /* mesh */
  Vector3d _pos = (Vector3d){0.5,0.0,-0.210};
  //Vector3d _pos = (Vector3d){0.5,0.0,-0.350};
  Matrix3d _rot = AngleAxisd(Dp::Math::deg2rad(90), Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto kawasaki_field = ssg::ImportObject("obj/field/ring_assy.stl", Eigen::Vector3d(0.001,0.001,0.001), _rot, _pos);
  scene.AddObject(kawasaki_field);

  /* TODO */
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
    } else if (i == 4 || i == 8 || i == 12 || i == 16) {
      geom[i] = new dSphere (*space, 0.00001);
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
  auto nearCb = static_cast<void(*)(void*data,dGeomID,dGeomID)>([](void *data, dGeomID o1, dGeomID o2) {
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
        //contact[i].surface.mode = dContactBounce; // 地面の反発係数を設定
        //contact[i].surface.mode = dContactBounce|dContactApprox1|dContactSoftERP|dContactSoftCFM;
        contact[i].surface.mode = dContactBounce|dContactApprox1|dContactSoftERP|dContactSoftCFM;
        contact[i].surface.mu   = 1.0; // dInfinity;
        contact[i].surface.soft_erp = 1.0; //1.0;
        contact[i].surface.soft_cfm = 1e-10;
        contact[i].surface.bounce = 0.00; // (0.0~1.0)   反発係数は0から1まで
        contact[i].surface.bounce_vel = 3.0; // (0.0以上)   反発に必要な最低速度
  
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
  //
  const dReal ref2 = -60.0;
  size_t count2[] = {1000,2500,4500,6000}; 
  dReal ref_angle_diff[] = {
      0.0, /* base */
      0.0,  Dp::Math::deg2rad(-ref2/2.0),  Dp::Math::deg2rad(ref2), 0.0,
      0.0,  Dp::Math::deg2rad(-ref2/2.0),  Dp::Math::deg2rad(ref2), 0.0,
      //0.0,                          0.0 ,                     0.0 , 0.0,
      //0.0,                          0.0 ,                     0.0 , 0.0,
      0.0, -Dp::Math::deg2rad(-ref2/2.0), -Dp::Math::deg2rad(ref2), 0.0,
      0.0, -Dp::Math::deg2rad(-ref2/2.0), -Dp::Math::deg2rad(ref2), 0.0
  };
  dReal ref_angle_diff2[] = {
      0.0, /* base */
      0.0,  Dp::Math::deg2rad(-45.0/2),  Dp::Math::deg2rad(45), 0.0,
      0.0,  Dp::Math::deg2rad(-45.0/2),  Dp::Math::deg2rad(45), 0.0,
      //0.0,                        0.0 ,                   0.0 , 0.0,
      //0.0,                        0.0 ,                   0.0 , 0.0,
      0.0, -Dp::Math::deg2rad(-45.0/2), -Dp::Math::deg2rad(45), 0.0,
      0.0, -Dp::Math::deg2rad(-45.0/2), -Dp::Math::deg2rad(45), 0.0
  };
  dReal ref_angle[nol];
  for (size_t j = 1; j < nol; j++) {
    auto rlink = robot->FindLink(lname[j]);
    ref_angle[j] = rlink->GetJoint().GetAngle();
  }

  /*** FTGL/SDL_ttf ***/
  TTF_Init();
  TTF_Font *font = TTF_OpenFont(fontfile, 30);

  if (!font) {
    std::cout << "ERROR can not open font : " << fontfile << std::endl;
  }
                         /* R G B A - */
  SDL_Color font_color = { 0, 0, 0, 255 }; /* ARGB 0xAARRGGBB */
  //SDL_Color font_bgcolor = { 0,0,0,255 }; /* ARGB */
  //SDL_Surface* tsurf = TTF_RenderUTF8_Shaded(font, "C++17er", font_color, font_bgcolor);
  std::string tmp_str;
  tmp_str = " GL : " + std::to_string(cmeasure.FPS()) + "[fps]";
  SDL_Surface* tsurf = TTF_RenderText_Blended(font, tmp_str.c_str(), font_color);
  tmp_str = "SIM : " + std::to_string(cmeasure.FPS()) + "[fps]";
  SDL_Surface* tsurf2 = TTF_RenderText_Blended(font, tmp_str.c_str(), font_color);

  //Create a surface to the correct size in RGB format, and copy the old image
  //                                                          depth      R          G         B          A
  //SDL_Surface * s = SDL_CreateRGBSurface(0, tsurf->w, tsurf->h, 32, 0x000000ff,0x0000ff00,0x00ff0000,0xff000000);
    // --> texobj->GetTexture().setdata(tsurf->w, tsurf->h, GL_RGBA, GL_UNSIGNED_BYTE /* NG: GL_UNSIGNED_INT_8_8_8_8 */, s->pixels);
  SDL_Surface * s = SDL_CreateRGBSurface(0, tsurf->w, tsurf->h, 32, 0x00ff0000,0x0000ff00,0x000000ff,0xff000000);
    // --> texobj->GetTexture().setdata(tsurf->w, tsurf->h, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8, s->pixels);
  //SDL_Surface * s = SDL_CreateRGBSurface(0, tsurf->w, tsurf->h, 32, 0xff000000,0x00ff0000,0x0000ff00,0xff); // RGBA
    // --> texobj->GetTexture().setdata(tsurf->w, tsurf->h, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8_REV, s->pixels);
  //SDL_Surface * s = SDL_CreateRGBSurface(0, tsurf->w, tsurf->h, 32, 0xff00, 0xff0000, 0xff000000, 0xff); // ARGB
    // --> texobj->GetTexture().setdata(tsurf->w, tsurf->h, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, s->pixels);
  SDL_BlitSurface(tsurf, NULL, s, NULL);

  printf(" %x(%d:%d) type:%x layout:%x, order:%x, %x R:%x G:%x B:%x A:%x\n", // ARGB  = 0xAARRGGBB
    tsurf->format->format, tsurf->format->BytesPerPixel, tsurf->format->BitsPerPixel,
    SDL_PIXELTYPE(tsurf->format->format), SDL_PIXELLAYOUT(tsurf->format->format), SDL_PIXELORDER(tsurf->format->format),
    SDL_BITSPERPIXEL(tsurf->format->format), tsurf->format->Rmask, tsurf->format->Gmask, tsurf->format->Bmask, tsurf->format->Amask); /* bits->20[bits] --> 2byte+4bit */
  printf(" %x(%d:%d) type:%x layout:%x, order:%x, %x R:%x G:%x B:%x A:%x\n", // ARGB  = 0xAARRGGBB
    s->format->format, s->format->BytesPerPixel, s->format->BitsPerPixel,
    SDL_PIXELTYPE(s->format->format), SDL_PIXELLAYOUT(s->format->format), SDL_PIXELORDER(s->format->format),
    SDL_BITSPERPIXEL(s->format->format), s->format->Rmask, s->format->Gmask, s->format->Bmask, s->format->Amask); /* bits->20[bits] --> 2byte+4bit */

  double aspect = tsurf->h / (double)(tsurf->w);
  Eigen::Matrix3f txtrot = AngleAxisf(Dp::Math::deg2rad(-90), Eigen::Vector3f::UnitY()).toRotationMatrix() *
                           AngleAxisf(Dp::Math::deg2rad( 90), Eigen::Vector3f::UnitZ()).toRotationMatrix();

  auto texobj = std::make_shared<SolidPlane>(txtrot, Eigen::Vector3f(0.0,0.0,0.3), 0.4, 0.4 * aspect);
  auto clr = Eigen::Vector4d(1.0,1.0,1.0,0.0);
  texobj->SetColor(clr);
  texobj->GetTexture().setdata(tsurf->w, tsurf->h, GL_BGRA, GL_UNSIGNED_BYTE, tsurf->pixels);
  scene.AddObject(texobj);

  auto texobj2 = std::make_shared<SolidPlane>(txtrot, Eigen::Vector3f(0.0,0.0,0.3 + 0.4 * aspect), 0.4, 0.4 * aspect);
  texobj2->SetColor(clr);
  texobj2->GetTexture().setdata(tsurf2->w, tsurf2->h, GL_BGRA, GL_UNSIGNED_BYTE, tsurf2->pixels);
  scene.AddObject(texobj2);

  auto texobj3 = std::make_shared<SolidPlane>(txtrot, Eigen::Vector3f(0.0,0.0,0.3 + 0.4 * aspect * 2), 0.4, 0.4 * aspect);
  texobj3->SetColor(clr);
  texobj3->GetTexture().setdata(s->w, s->h, GL_BGRA, GL_UNSIGNED_BYTE, s->pixels);
  scene.AddObject(texobj3);

  uint32_t* buff = (uint32_t*)tsurf->pixels;
  uint32_t* buff2 = (uint32_t*)s->pixels;
  for (ssize_t i = 0; i < tsurf->w; i++) {
    printf("---\n");
    for (ssize_t j = 0; j < tsurf->h; j++) {
      printf(" %08x", buff[i*tsurf->h + j]);
    }
    printf("\n");
    for (ssize_t j = 0; j < tsurf->h; j++) {
      printf(" %08x", buff2[i*tsurf->h + j]);
    } 
    printf("\n");
  }
  for (ssize_t i = 0; i < tsurf->h; i++) {
    for (ssize_t j = 0; j < tsurf->w; j++) {
      if ((buff[i*tsurf->w + j] & 0xFF000000) == 0xFF000000) {
        printf("*");
      } else if ((buff[i*tsurf->w + j] & 0xFF000000)) {
        printf(".");
      } else {
        printf(" ");
      }
    }
    printf("\n");
  }
  for (ssize_t i = 0; i < tsurf->h; i++) {
    for (ssize_t j = 0; j < tsurf->w; j++) {
      if ((buff[i*tsurf->w + j] & 0xFF000000) == 0xFF000000) {
        printf("*");
      } else if (buff2[i*tsurf->w + j] & 0xFF000000) {
        printf(".");
      } else {
        printf(" ");
      }
    }
    printf("\n");
  }

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<int> ang_diff(-1.0,1.0);

  size_t count = 0;
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

    ode_link.World2Link(world);
    khr3->UpdateCasCoords();

    /* TODO: body centre */
    sphere[0]->SetOffset(robot->WPos(), robot->WRot());

    /* KHR3-HV */
#if 0
    for (size_t j = 1; j < nok; j++) {
      auto rlink = khr3->FindLink(khr_lname[j]);
      if (!rlink) {
        std::cout << khr_lname[j] << std::endl;
        break;
      }
      rlink->GetJoint().SetValue(rlink->GetJoint().GetAngle() + Dp::Math::deg2rad(ang_diff(mt)));
    }
    khr3->UpdateCasCoords();
#endif

    //scene.Draw();

    cmeasure.update();

    std::string str;
    str = "SIM : " + std::to_string(cmeasure.FPS() * (WORLD_STEP/0.001)) + "[fps]";
    SDL_FreeSurface(tsurf);
    //tsurf = TTF_RenderUTF8_Blended(font, str.c_str(), font_color);
    tsurf = TTF_RenderText_Blended(font, str.c_str(), font_color);
    SDL_FreeSurface(tsurf2);
    str = " GL : " + std::to_string(scene.FPS()) + "[fps]";
    tsurf2 = TTF_RenderText_Blended(font, str.c_str(), font_color);

    texobj->GetTexture().setdata(tsurf->w, tsurf->h, GL_BGRA, GL_UNSIGNED_BYTE, tsurf->pixels);
    texobj2->GetTexture().setdata(tsurf2->w, tsurf2->h, GL_BGRA, GL_UNSIGNED_BYTE, tsurf2->pixels);
    
    /********* KHR controller side *************************/
    {
      std::shared_ptr<dJoint> jnt;
      jnt = ode_link.FindJoint(khr_lname[5]);
      if (jnt) {
        jnt->setParam(dParamVel, -200);
        jnt->setParam(dParamFMax, Dp::Phyx::Kgcm2Nm(20.0));
      }
      jnt = ode_link.FindJoint(khr_lname[7]);
      if (jnt) {
        jnt->setParam(dParamVel, +200);
        jnt->setParam(dParamFMax, Dp::Phyx::Kgcm2Nm(20.0));
      }
    }

    /********* eV controller side *************************/
    double Kp = 30;
    // [kgf-cm]
    const double KRS2572HV_MAX_TRQ  = 25.0 * (13.2/11.1) * 100;
    //const double KRS6003RHV_MAX_TRQ = 67.0 * (13.2/11.1);
    // [rad/sec] <-- [deg/sec] <-- 0.13[sec/60deg]
    const double KRS2572HV_MAX_SPD  = Dp::Math::deg2rad(60.0 / 0.13) * (13.2/11.1) * 100;
    //const double KRS6003RHV_MAX_SPD = Dp::Math::deg2rad(60.0 / 0.22) * (13.2/11.1);
#define TARGET_MAX_TRQ KRS2572HV_MAX_TRQ
//#define TARGET_MAX_TRQ KRS6003RHV_MAX_TRQ
#define TARGET_MAX_SPD KRS2572HV_MAX_SPD
//#define TARGET_MAX_SPD KRS6003RHV_MAX_SPD
    bool islimit = false;
    for (size_t j = 1; j < nol; j++) {
      auto rlink = robot->FindLink(lname[j]);
      double diff;
      if (count < count2[0]) {
        diff = ref_angle[j] - rlink->GetJoint().GetAngle();
      } else if (count < count2[1]) {
        diff = ref_angle[j] - rlink->GetJoint().GetAngle() + ref_angle_diff[j];
      } else if (count < count2[2]) {
        diff = ref_angle[j] - rlink->GetJoint().GetAngle() + ref_angle_diff2[j];
      } else if (count < count2[3]) {
        diff = ref_angle[j] - rlink->GetJoint().GetAngle() + ref_angle_diff2[j];
      } else {
        diff = ref_angle[j] - rlink->GetJoint().GetAngle();
      }
      double cspd = Kp * diff;
      if (cspd < 0) {
        if (-cspd > (TARGET_MAX_SPD)){
          cspd = -(TARGET_MAX_SPD);
          //islimit = true;
        }
      } else {
        if ( cspd > TARGET_MAX_SPD){
          cspd =  TARGET_MAX_SPD;
          //islimit = true;
        }
      }
      joint[j]->setParam(dParamVel, -cspd);
      joint[j]->setParam(dParamFMax, Dp::Phyx::Kgcm2Nm(TARGET_MAX_TRQ));
      if (islimit) {
        printf(" %+7.2lf", Dp::Math::rad2deg(cspd));
        if (j == 4 || j == 8 || j == 12 || j == 16) {
          printf(" (max:%+7.2lf) [deg/s]\n", Dp::Math::rad2deg(TARGET_MAX_SPD));
        }
      }
    }

    bool ismax = false;
    static double jtrq_max[20] = {0.0};
    for (size_t i = 1; i < nol; i++){
      dJointFeedback* jf;
      jf = joint[i]->getFeedback();
      if (jf == NULL) continue;

      if (jtrq_max[i]*jtrq_max[i] < jf->t2[1]*jf->t2[1]) {
        jtrq_max[i] = jf->t2[1]; 
        ismax = true;
      }
      /*
      printf("[%02zd] : %+7.2lf %+7.2lf %+7.2lf %+7.2lf %+7.2lf %+7.2lf\n",
          i,
          jf->t1[0], jf->t1[1], jf->t1[2],
          jf->t2[0], jf->t2[1], jf->t2[2]);
      */
    }
    ismax = false;
    if (ismax) {
      for (size_t i = 1; i < nol; i++){
        //printf(" %+012.4lf", Dp::Phyx::Nm2Kgcm(jtrq_max[i]));
        printf(" %+012.4lf", (jtrq_max[i]));
        if (i == 4 || i == 8 || i == 12 || i == 16) {
          printf("\n");
        }
      }
      printf("\n");
    }
#if 0
    for (size_t i = 1; i < nol; i++){
      dJointFeedback* jf;
      jf = joint[i]->getFeedback();
      //printf("%+8.3lf", (jf->t2[1]));
      printf("%+8.3lf", Dp::Phyx::Nm2Kgcm(jf->t2[1]));
      if (i < nol - 1) {
        printf(",");
      }
    }
    printf("\n");
#endif
    //printf("%lf, %lf, %lf\n", robot->WPos()(0), robot->WPos()(1), robot->WPos()(2));

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

    for (size_t i = 0; i < nok; i++) {
      auto rlink = khr3->FindLink(khr_lname[i]);
      if (!rlink) continue;
      ksphere[i]->SetOffset(rlink->GetWCentroid(), rlink->WRot());
    }
   
    space->collide((void*)&coldata, nearCb);
    world.step(WORLD_STEP);
    jgrp.empty();

#endif
    count++;
  }

  TTF_CloseFont(font);
  TTF_Quit();

  return 0;
}

