
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

namespace ssg {
  class UniMesh : public UniPartedObject {
  private:
  
  public:
    UniMesh (const Eigen::Matrix3f &rot, const aiMesh* paiMesh) {
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
      vertices_.rotate(rot);
  
      /* make indices */
      for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        //printf("%d, %d, %d\n", Face.mIndices[0], Face.mIndices[1], Face.mIndices[2]);
  
        indices_.push_back(Face.mIndices[0]);
        indices_.push_back(Face.mIndices[1]);
        indices_.push_back(Face.mIndices[2]);
      }
  
      BuildObject();
    }

    UniMesh (const aiMesh* paiMesh) :UniMesh(Eigen::Matrix3f::Identity(), paiMesh){}
  };

  std::shared_ptr<SceneObject> ImportObject (std::string file_name) {
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
      return std::make_shared<UniMesh>(pScene->mMeshes[0]);
    }

    printf("NumMeshes > 1 not supported\n");
    return NULL;
  }

  void test() {
    Assimp::Importer Importer;
  
    std::string Filename = "./test.stl";
    printf("------------ %s\n", Filename.c_str());
  
    const aiScene* pScene = 
      Importer.ReadFile(Filename.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
    // aiProcess_FlipUVs
  
    DPRINTF("result '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
    if (pScene) {
      //Ret = InitFromScene(pScene, Filename);
      printf("result '%s': \n", Filename.c_str());
    }
    else {
      printf("Error parsing '%s': '%s'\n", Filename.c_str(), Importer.GetErrorString());
      return;
    }
  
    printf(" mesh size : %u\n", pScene->mNumMeshes);
    printf(" mate size : %u\n", pScene->mNumMaterials);
  
    const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
   
    for (unsigned int i = 0 ; i < pScene->mNumMeshes ; i++) {
      const aiMesh* paiMesh = pScene->mMeshes[i];
      //InitMesh(i, paiMesh);
  
      for (unsigned int i = 0 ; i < paiMesh->mNumVertices ; i++) {
        const aiVector3D* pPos      = &(paiMesh->mVertices[i]);
        const aiVector3D* pNormal   = &(paiMesh->mNormals[i]);
        const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &Zero3D;
  
        printf("%+7.2lf, %+7.2lf, %+7.2lf : %+7.2lf, %+7.2lf || %+7.2lf, %+7.2lf, %+7.2lf\n",
          pPos->x, pPos->y, pPos->z, pTexCoord->x, pTexCoord->y, pNormal->x, pNormal->y, pNormal->z);
        //Vertex v(Vector3f(pPos->x, pPos->y, pPos->z),
        //         Vector2f(pTexCoord->x, pTexCoord->y),
        //         Vector3f(pNormal->x, pNormal->y, pNormal->z));
        //
        //Vertices.push_back(v);
      }
  
      for (unsigned int i = 0 ; i < paiMesh->mNumFaces ; i++) {
        const aiFace& Face = paiMesh->mFaces[i];
        assert(Face.mNumIndices == 3);
        printf("%d, %d, %d\n", Face.mIndices[0], Face.mIndices[1], Face.mIndices[2]);
        //Indices.push_back(Face.mIndices[0]);
        //Indices.push_back(Face.mIndices[1]);
        //Indices.push_back(Face.mIndices[2]);
      }
    }
  
    //return InitMaterials(pScene, Filename);
  }
}

#endif

