
#ifndef DP_TYPE_H
#define DP_TYPE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

typedef int32_t errno_t;

namespace Dp {

  namespace Math {
    typedef double real;

    const real PI = 3.14159265359;
    const real PIDEG = 180.0;

    inline const real rad2deg (const real rad) {
      return rad * PIDEG / PI;
    }

    inline const real deg2rad (const real deg) {
      return deg * PI / PIDEG;
    }

    inline Eigen::Matrix3d rpy2mat3 (Eigen::Vector3d rpy) {
  
      //Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
      //Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
      //Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
      //
      //Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
      //
      //Eigen::Matrix3d rotationMatrix = q.matrix();
      
      auto rangle = Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());
      auto pangle = Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY());
      auto yangle = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ());
      auto rpyangle = rangle * pangle * yangle;
  
      return rpyangle.toRotationMatrix();
    }
  }

  namespace Phyx {
    const Math::real G = 9.8;

    inline const Math::real Nm2Kgcm (Math::real moment) {
      return moment * 100.0 / G; /* Nm --> Ncm --> kgcm */
    }

    inline const Math::real Kgcm2Nm (Math::real kgcm) {
      return kgcm * G / 100.0;   /* Nm --> Ncm --> kgcm */
    }
  }

}

#endif

