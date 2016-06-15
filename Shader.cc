//#include <stdlib.h>
//#include <stdio.h>

//#include <time.h> // for clock_gettime()
//#include <cstring>
//#include "cycle_measure.h"

#include <list>
#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "dp_type.h"

namespace ssg {

  const GLchar* readShaderSource(const char *file)
  {
    FILE *fp;
    GLchar *source;
    GLsizei length;
    int ret;
    
    /* ファイルを開く */
    fp = fopen(file, "rb");
    if (fp == NULL) {
      perror(file);
      return NULL;
    }
    
    /* ファイルの末尾に移動し現在位置 (つまりファイルサイズ) を得る */
    fseek(fp, 0L, SEEK_END);
    length = ftell(fp);
    
    /* ファイルサイズのメモリを確保 */
    source = (GLchar *)malloc(length + 1);
    if (source == NULL) {
      fprintf(stderr, "Could not allocate read buffer.\n");
      return NULL;
    }
    
    /* ファイルを先頭から読み込む */
    fseek(fp, 0L, SEEK_SET);
    ret = fread((void *)source, 1, length, fp) != (size_t)length;
    fclose(fp);
    
    /* シェーダのソースプログラムのシェーダオブジェクトへの読み込み */
    if (ret) {
      fprintf(stderr, "Could not read file: %s.\n", file);
      return NULL;
    }
    //else
    //  glShaderSource(shader, 1, &source, &length);
    
    /* 確保したメモリの開放 */
    //free((void *)source);
    source[length] = '\0';
    //printf("%x %x %x\n", source[length -2], source[length - 1], source[length]);
    
    return source;
  }
  
  GLboolean printShaderInfoLog(GLuint shader, const char*str) {
    //コンパイル結果を取得する
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE) {
      //std::cerr << "Compile Error in " << str << std::endl;
      fprintf(stderr, "Compile Error %s\n", str);
    }
    //シェーダのコンパイル時のログの長さを取得する
    GLsizei bufSize;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH , &bufSize);
    if (bufSize > 1) {
      //シェーダのコンパイル時のログの内容を取得する
      //std::vector<GLchar> infoLog(bufSize);
      char infoLog[bufSize + 1];
      GLsizei length;
      glGetShaderInfoLog(shader, bufSize, &length, &infoLog[0]);
      //std::cerr << &infoLog[0] << std::endl;
      fprintf(stderr, "%s\n", &infoLog[0]);
    }
    //return static_cast<GLboolean>(status);
    return (GLboolean)(status);
  }
  
  GLboolean printProgramInfoLog(GLuint program) {
    //リンク結果を取得する
    GLint status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE) {
      //std::cerr << "Link Error." << std::endl;
      fprintf(stderr, "Link Error\n");
    }
     //シェーダのリンク時のログの長さを取得する
    GLsizei bufSize;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufSize);
  
    if (bufSize > 1) {
      //シェーダのリンク時のログの内容を取得する
      //std::vector<GLchar> infoLog(bufSize);
      char infoLog[bufSize + 1];
      GLsizei length;
      glGetProgramInfoLog(program, bufSize, &length, &infoLog[0]);
      //std::cerr << &infoLog[0] << std::endl;
      fprintf(stderr, "%s\n", &infoLog[0]);
    }
    //return static_cast<GLboolean>(status);
    return (GLboolean)(status);
  }
  
  GLuint createProgram (const char *vsrc, const std::list<std::string> attrs, const char *fsrc, const char *fc)
  {
  
    //空のプログラムオブジェクトを作成する
    //const GLuint program(glCreateProgram());
    const GLuint program = glCreateProgram();
  
    if (vsrc != NULL) {
      //バーテックスシェーダのシェーダオブジェクトを作成する
      //const GLuint vobj(glCreateShader(GL_VERTEX_SHADER));
      const GLuint vobj = glCreateShader(GL_VERTEX_SHADER);
  
      glShaderSource(vobj, 1, &vsrc, NULL);
      glCompileShader(vobj);
  
      //バーテックスシェーダのシェーダオブジェクトをプログラムオブジェクトに組み込む
      if (printShaderInfoLog(vobj, "vertex shader")) {
        glAttachShader(program, vobj);
      }
  
      glDeleteShader(vobj);
    }
  
    if (fsrc != NULL) {
      //フラグメントシェーダのシェーダオブジェクトを作成する
      //const GLuint fobj(glCreateShader(GL_FRAGMENT_SHADER));
      const GLuint fobj = glCreateShader(GL_FRAGMENT_SHADER);
  
      glShaderSource(fobj, 1, &fsrc, NULL);
      glCompileShader(fobj); 
  
      //フラグメントシェーダのシェーダオブジェクト
      //をプログラムオブジェクトに組み込む
      if (printShaderInfoLog(fobj, "fragment shader")) {
        //glAttachShader(program, vobj);
        glAttachShader(program, fobj);
      }
      glDeleteShader(fobj);
    }
  
    ////
    //プログラムオブジェクトをリンクする
    GLuint idx = 0;
    for (auto &attr : attrs) {
      glBindAttribLocation(program, idx, attr.c_str());
      printf("glBindAttribLocation: %u %u %s\n", program, idx, attr.c_str());
      idx++;
    }
    //glBindAttribLocation(program, 0, pv);
    glBindFragDataLocation(program, 0, fc);
  
    glLinkProgram(program);
  
    if (printProgramInfoLog(program)) {
      return program;
    }
  
    ////
    //作成したプログラムオブジェクトを返す
    glDeleteProgram(program);
    return 0;
  }
  
  GLuint loadProgram (const char *vert, const std::list<std::string> &attrs, const char *frag, const char *fc) {
    //シェーダのソースファイルを読み込む
    const GLchar *vsrc = (readShaderSource(vert));
    const GLchar *fsrc = (readShaderSource(frag));
    // プログラムオブジェクトを作成する
    const GLuint program = (createProgram(vsrc, attrs, fsrc, fc));
    // 読み込みに使ったメモリを解放する
    free((GLchar*)vsrc);
    free((GLchar*)fsrc);
    //delete vsrc;
    //delete fsrc;
    // 作成したプログラムオブジェクトを返す
    return program;
  }

  errno_t loadProgram2 (GLuint program, const char *vert, const std::list<std::string> &attrs, const char *frag, const char *fc) {
    //シェーダのソースファイルを読み込む
    const GLchar *vsrc = (readShaderSource(vert));
    const GLchar *fsrc = (readShaderSource(frag));
    // プログラムオブジェクトを作成する
    program = (createProgram(vsrc, attrs, fsrc, fc));
    // 読み込みに使ったメモリを解放する
    free((GLchar*)vsrc);
    free((GLchar*)fsrc);
    //delete vsrc;
    //delete fsrc;
    // 作成したプログラムオブジェクトを返す
    return 0;
  }
}

