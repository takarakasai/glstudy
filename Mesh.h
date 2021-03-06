
#ifndef MESH_H
#define MESH_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "dp_type.h"
#include "PartedObject.h"
#include "PrimitiveObject.h"

namespace ssg {

  class SolidMesh : public UniPartedObject {
  private:
  
  public:
    SolidMesh (const aiMesh* paiMesh, const Eigen::Vector3d &scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) {
      /* make vertices */
      const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
      for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
        const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
        const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);

        if (paiMesh->HasTextureCoords(0)) {
          const aiVector3D* pTexCoord = &(paiMesh->mTextureCoords[0][i]);
          vertices_.push(
              Eigen::Vector3f(pPos->x, pPos->y, pPos->z),
              Eigen::Vector3f(pNormal->x, pNormal->y, pNormal->z),
              Eigen::Vector2f(pTexCoord->x, pTexCoord->y));
        } else {
          vertices_.push(
              Eigen::Vector3f(pPos->x, pPos->y, pPos->z),
              Eigen::Vector3f(pNormal->x, pNormal->y, pNormal->z));
        }
        // printf("%lf %lf %lf\n",pNormal->x, pNormal->y, pNormal->z);
        //printf("%+7.2lf, %+7.2lf, %+7.2lf : %+7.2lf, %+7.2lf || %+7.2lf, %+7.2lf, %+7.2lf -- %s\n",
        //  pPos->x, pPos->y, pPos->z, pTexCoord->x, pTexCoord->y, pNormal->x, pNormal->y, pNormal->z,
        //  paiMesh->HasTextureCoords(0) ? "True" : "False");
      }
      vertices_.scale(scale.cast<float>());
      vertices_.rotate(rot.cast<float>());
      vertices_.offset(pos.cast<float>());
  
      /* make indices */
      for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        //printf("%d, %d, %d\n", Face.mIndices[0], Face.mIndices[1], Face.mIndices[2]);
  
        indices_.push_back(Face.mIndices[0]);
        indices_.push_back(Face.mIndices[1]);
        indices_.push_back(Face.mIndices[2]);
      }
  
      //BuildObject();
      BuildObject(GL_TRIANGLES);
    }

    //SolidMesh (const aiMesh* paiMesh, Dp::Math::real scale = 1.0) :SolidMesh(paiMesh, scale, Eigen::Matrix3f::Identity()){}
    SolidMesh (const aiMesh* paiMesh, Dp::Math::real scale = 1.0) :
        SolidMesh(paiMesh, scale, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()){}
    SolidMesh (const aiMesh* paiMesh, const Dp::Math::real scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) :
        SolidMesh(paiMesh, Eigen::Vector3d(scale, scale, scale), rot, pos) {};
  };

  class WiredMesh : public UniPartedObject {
  private:
  
  public:
    WiredMesh (const aiMesh* paiMesh, const Eigen::Vector3d &scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) {
      /* make vertices */
      const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
      for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
        const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
        const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);
        const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;
 
        vertices_.push(
            Eigen::Vector3f(pPos->x, pPos->y, pPos->z),
            Eigen::Vector3f(pNormal->x, pNormal->y, pNormal->z),
            Eigen::Vector2f(pTexCoord->x, pTexCoord->y));
        //printf("%+7.2lf, %+7.2lf, %+7.2lf : %+7.2lf, %+7.2lf || %+7.2lf, %+7.2lf, %+7.2lf -- %s\n",
        //  pPos->x, pPos->y, pPos->z, pTexCoord->x, pTexCoord->y, pNormal->x, pNormal->y, pNormal->z,
        //  paiMesh->HasTextureCoords(0) ? "True" : "False");
      }
      vertices_.scale(scale.cast<float>());
      vertices_.rotate(rot.cast<float>());
      vertices_.offset(pos.cast<float>());
  
      /* make indices */
      for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        //printf("%d, %d, %d\n", Face.mIndices[0], Face.mIndices[1], Face.mIndices[2]);
  
        indices_.push_back(Face.mIndices[0]);
        indices_.push_back(Face.mIndices[1]);

        indices_.push_back(Face.mIndices[1]);
        indices_.push_back(Face.mIndices[2]);

        indices_.push_back(Face.mIndices[2]);
        indices_.push_back(Face.mIndices[0]);
      }
  
      BuildObject(GL_LINES);
      //BuildObject(GL_TRIANGLES);
    }

    WiredMesh (const aiMesh* paiMesh, Dp::Math::real scale = 1.0) :
        WiredMesh(paiMesh, scale, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()){}
    WiredMesh (const aiMesh* paiMesh, const Dp::Math::real scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) :
        WiredMesh(paiMesh, Eigen::Vector3d(scale, scale, scale), rot, pos) {};
  };

#if 0
  std::shared_ptr<Mesh> ImportObject (
          std::string file_name, const Dp::Math::real scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) {
    Assimp::Importer Importer;
  
    const aiScene* pScene = 
      Importer.ReadFile(file_name.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
    // aiProcess_FlipUVs
  
    if (pScene == NULL) {
      printf("Error parsing '%s': '%s'\n", file_name.c_str(), Importer.GetErrorString());
      return NULL;
    }

    printf(" mesh size : %u\n", pScene->mNumMeshes);
    printf(" mate size : %u\n", pScene->mNumMaterials);

    if (pScene->mNumMeshes == 1) {
      printf("NumMeshes == 1 ==> UniMesh\n");
      return std::make_shared<Mesh>(pScene->mMeshes[0], scale, rot, pos);
    }

    printf("NumMeshes > 1 not supported\n");
    return NULL;
  }
#endif
}

template<> SwitchableSceneObject<ssg::WiredMesh, ssg::SolidMesh>::SwitchableSceneObject (
        const aiMesh* paiMesh, const Eigen::Vector3d &scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos)
  : mode_(SOLID), wired_(paiMesh, scale, rot, pos), solid_(paiMesh, scale, rot, pos) {
}

typedef SwitchableSceneObject<ssg::WiredMesh       , ssg::SolidMesh       > Mesh;

namespace ssg {

  std::shared_ptr<Mesh> ImportObject (
          std::string file_name, const Eigen::Vector3d &scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) {
    Assimp::Importer Importer;
  
    const aiScene* pScene = 
      Importer.ReadFile(file_name.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
    // aiProcess_FlipUVs
  
    if (pScene == NULL) {
      printf("Error parsing '%s': '%s'\n", file_name.c_str(), Importer.GetErrorString());
      return NULL;
    }

    printf(" mesh size : %u\n", pScene->mNumMeshes);
    printf(" mate size : %u\n", pScene->mNumMaterials);

    if (pScene->mNumMeshes == 1) {
      printf("NumMeshes == 1 ==> UniMesh\n");
      return std::make_shared<Mesh>(pScene->mMeshes[0], scale, rot, pos);
    }

    printf("NumMeshes > 1 not supported\n");
    return NULL;
  }
}

#endif

