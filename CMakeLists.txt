
cmake_minimum_required(VERSION 2.8)

if(APPLE)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)
else()
set(CMAKE_CXX_COMPILER g++)
endif()

# dDOUBLE & dIDEDOUBLE to be set for using ODE with double precision
#add_definitions(-std=c++11 -Wall -DdDOUBLE -DdIDEDOUBLE)
add_definitions(-std=c++11 -Wall -DdDOUBLE)

include_directories(
  .
)

#add_definitions(-D_DEBUG)


if(APPLE)
  find_library( OpenGL_LIBRARY OpenGL )
  set(EXTRA_LIBS glfw3 ${OpenGL_LIBRARY})
else()
    #set(EXTRA_LIBS glfw GL)
  set(EXTRA_LIBS glfw GL ode)
endif()

link_libraries(
  GLEW ${EXTRA_LIBS} assimp
)

if(LINUX)
  link_libraries(GL)
endif()

file(GLOB SRCS "./*.cc")

message(INFO ${SRCS})

add_executable(
  ssg_test
  ${SRCS}
)

execute_process(
  COMMAND rm ${CMAKE_BINARY_DIR}/obj
  COMMAND ln -s ${CMAKE_SOURCE_DIR}/obj ${CMAKE_BINARY_DIR}/obj
  COMMAND rm ${CMAKE_BINARY_DIR}/shader
  COMMAND ln -s ${CMAKE_SOURCE_DIR}/shader ${CMAKE_BINARY_DIR}/shader
)

#add_custom_command (
#  OUTPUT filename
#  COMMAND
#  DEPENDS 
#  )

