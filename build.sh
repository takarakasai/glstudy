#! /bin/bash

if [ "$(uname)" == "Darwin" ]; then
  echo "-- OSX"
  #g++ test.cc --std=c++11 -framework OpenGL -lglfw3 -lGLEW -o test.out
  c++ test.cc --std=c++11 -framework OpenGL -lglfw3 -lGLEW -o test.out
elif [ "$(uname)" == "Linux" ]; then
  echo "-- Linux"
  g++ test.cc --std=c++11 -lglfw3 -lGLEW -lGL -o test.out
else
  echo "-- not supported"
fi

