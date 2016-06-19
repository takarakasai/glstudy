#ifndef DRAWABLELINK_H
#define DRAWABLELINK_H

#include <list>

#include "dp_type.h"

#include "Link.h"
#include "InterfaceSceneObject.h"

using namespace Eigen;

/* ssg : Simple Scene Graph */
namespace ssg {

  class DrawableLink : public Link, public InterfaceSceneObject/*Implement*/ {
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

  public: /* InterfaceSceneObject */
    errno_t Exec(void) {
      for (auto &obj : objs_) {
        obj->Draw(CasCoords::WRot(), CasCoords::WPos());
      }
      return 0;
    }
    errno_t Draw(void) {
      ECALL(ExecAll());
      return 0;
    }

    errno_t Draw(Eigen::Matrix3d& rot, Eigen::Vector3d& pos) {
      return -1;
    }

    errno_t SetTransformMatrixLocId (int32_t id) {
      for (auto &obj : objs_) {
        obj->SetTransformMatrixLocId(id);
      }
      for (auto &link : clinks_) {
        link->SetTransformMatrixLocId(id);
      }
      return 0;
    }

    errno_t SetMaterialColorLocId (int32_t id) {
      for (auto &obj : objs_) {
        obj->SetMaterialColorLocId(id);
      }
      for (auto &link : clinks_) {
        link->SetMaterialColorLocId(id);
      }
      return 0;
    }

    errno_t SetOffset(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
      return -1;
    }

    errno_t SetColor(Eigen::Vector4d& color) {
      return -1;
    }

    errno_t SetDrawMode (InterfaceSceneObject::DrawMode mode) {
      for (auto &obj : objs_) {
        obj->SetDrawMode(mode);
      }
      for (auto &link : clinks_) {
        link->SetDrawMode(mode);
      }

      return 0;
    }

    Coordinates& GetCoordinates() {
      return CasCoords::World();
    }

  };
}

#endif

