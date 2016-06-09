#! /bin/bash

if [ "$(uname)" == "Darwin" ]; then
  echo "-- OSX"
  #g++ test.cc --std=c++11 -framework OpenGL -lglfw3 -lGLEW -o test.out
  c++ test.cc --std=c++11 -framework OpenGL -lglfw3 -lGLEW -o test.out
elif [ "$(uname)" == "Linux" ]; then
  echo "-- Linux"
  g++ test.cc --std=c++11 -lglfw -lGLEW -lGL -lassimp -o test.out
else
  echo "-- not supported"
fi

# real    0m6.149s
# user    0m5.865s
# sys 0m0.261s

