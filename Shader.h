//#include <stdlib.h>
//#include <stdio.h>

//#include <time.h> // for clock_gettime()
//#include <cstring>
//#include "cycle_measure.h"

#ifndef SHADER_H
#define SHADER_H

#include <list>
#include <string>

//#include <GL/glew.h>
//#include <GLFW/glfw3.h>

#include "dp_type.h"

namespace ssg {

  /*
  ** シェーダーのソースプログラムをメモリに読み込む
  */
  const GLchar* readShaderSource(const char *file);
 
  //シェーダオブジェクトのコンパイル結果を表示する
  // shader:シェーダオブジェクト名
  // str:コンパイルエラーが発生した場所を示す文字列
  GLboolean printShaderInfoLog(GLuint shader, const char*str);
 
  //プログラムオブジェクトのリンク結果を表示する
  // program:プログラムオブジェクト名
  GLboolean printProgramInfoLog(GLuint program);
 
  
  //プログラムオブジェクトを作成する
  // vsrc: バーテックスシェーダのソースプログラムの文字列
  // pv: バーテックスシェーダのソースプログラム中のin変数名の文字列
  // fsrc: フラグメントシェーダのソースプログラムの文字列
  // fc: フラグメントシェーダのソースプログラム中のout変数名の文字列
  GLuint createProgram (const char *vsrc, const std::list<std::string> attrs, const char *fsrc, const char *fc);
 
  // シェーダのソースファイルを読み込んでプログラムオブジェクトを作成する
  // vert: バーテックスシェーダのソースファイル名
  // pv: バーテックスシェーダのソースプログラム中のin変数名の文字列
  // frag: フラグメントシェーダのソースファイル名
  // fc: フラグメントシェーダのソースプログラム中のout変数名の文字列
  GLuint loadProgram (const char *vert, const std::list<std::string> &attrs, const char *frag, const char *fc);

}

#endif

