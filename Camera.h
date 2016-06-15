
#include <eigen3/Eigen/Core>

#include <GLFW/glfw3.h>

#include "dp_type.h"

namespace ssg {

  class Camera {
    typedef Dp::Math::real Real;
  private:
    Eigen::Vector3d pos;
    Eigen::Vector3d dir_to;
    Eigen::Vector3d top;

  public:
    Camera(Real px, Real py, Real pz,
           Real dx, Real dy, Real dz,
           Real tx, Real ty, Real tz):
      pos(px, py, pz), dir_to(dx, dy, dz), top(tx, ty, tz) {
    }
    virtual ~Camera() {};

    Eigen::Vector3d Dir() {
      return dir_to - pos;
    }

    Eigen::Vector3d LeftDir() {
      return top.cross(Dir());
    }

    Eigen::Matrix4d LookAtMatrix() {
      //std::cout << "POS:" <<  pos << std::endl;
      //std::cout << "DIR:" <<  dir_to << std::endl;
      return ssg::lookAt(pos, dir_to, top);
    }

    Eigen::Matrix4d ProjectionMatrix() {
      Eigen::Matrix4d camera_mat = ssg::cameraPerspectiveMatrix(90.0, 1.0, 0.05, 2.0);
      Eigen::Matrix4d projection_mat = LookAtMatrix() * camera_mat;
      return projection_mat;
    }

    Camera& operator+=(Real dpos) {
      Eigen::Vector3d dir = Dir();
      std::cout << "DIR:" <<  dir << "   :" << dpos << std::endl;
      pos += dir * dpos;
      dir_to += dir * dpos;
      return (*this);
    }

    Camera& operator-=(Real dpos) {
      Eigen::Vector3d dir = Dir();
      pos -= dir * dpos;
      dir_to -= dir * dpos;
      return (*this);
    }

    Camera& operator+=(Eigen::Vector3d dpos) {
      pos += dpos;
      dir_to += dpos;
      return (*this);
    }

    Camera& operator-=(Eigen::Vector3d dpos) {
      pos -= dpos;
      dir_to -= dpos;
      return (*this);
    }

    void Rotate(Real r, Real p, Real y) {
      Eigen::Vector3d rpy(r, p, y);
      Eigen::Vector3d dir = Dir();
      dir_to = Dp::Math::rpy2mat3(rpy) * dir + pos;
      return;
    }
  };

}
