
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
    SolidMesh (const aiMesh* paiMesh, const Dp::Math::real scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) {
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
        // printf("%lf %lf %lf\n",pNormal->x, pNormal->y, pNormal->z);
        //printf("%+7.2lf, %+7.2lf, %+7.2lf : %+7.2lf, %+7.2lf || %+7.2lf, %+7.2lf, %+7.2lf -- %s\n",
        //  pPos->x, pPos->y, pPos->z, pTexCoord->x, pTexCoord->y, pNormal->x, pNormal->y, pNormal->z,
        //  paiMesh->HasTextureCoords(0) ? "True" : "False");
      }
      vertices_.scale(scale);
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
  };

  class WiredMesh : public UniPartedObject {
  private:
  
  public:
    WiredMesh (const aiMesh* paiMesh, const Dp::Math::real scale, const Eigen::Matrix3d &rot, const Eigen::Vector3d &pos) {
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
      vertices_.scale(scale);
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

    WiredMesh (const aiMesh* paiMesh, Dp::Math::real scale = 1.0) :WiredMesh(paiMesh, scale, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()){}
  };

  typedef SwitchableSceneObject<WiredMesh       , SolidMesh       > Mesh;

  std::shared_ptr<SolidMesh> ImportObject (
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
      return std::make_shared<SolidMesh>(pScene->mMeshes[0], scale, rot, pos);
    }

    printf("NumMeshes > 1 not supported\n");
    return NULL;
  }
}

#endif

