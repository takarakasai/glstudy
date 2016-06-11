#ifndef OBJ_FILE_READER_H
#define OBJ_FILE_READER_H

//#include <stdlib.h>
#include <stdio.h> //#include <cstdlib>
#include <list>
#include <string>

#include "dp_type.h"

#include "Link.h"
#include "DrawableLink.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <functional>

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
      //dynamic_cast<DrawableLink&>(link).AddShape(shapes);
      link.AddShape(shapes);

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
      auto rot = Dp::Math::rpy2mat3(rpy);
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

    //auto shape = std::make_shared<WiredCylinder>(
    auto shape = std::make_shared<Cylinder>(
      Eigen::AngleAxisf(Dp::Math::deg2rad(90), (Vector3f){1,0,0}).toRotationMatrix(),  Eigen::Vector3f::Zero(), radius, height, (size_t)20);

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

    auto shape = std::make_shared<Rectangular>(
        Eigen::AngleAxisf(Dp::Math::deg2rad(0), (Vector3f){0,0,0}).toRotationMatrix(),
        Eigen::Vector3f::Zero(), lx, ly, (float)lz);
    //auto shape = std::make_shared<WiredRectangular>(Eigen::Vector3f::Zero(), lx, ly, lz);

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

#endif

