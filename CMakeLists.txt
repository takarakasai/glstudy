
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_COMPILER /usr/bin/clang++)

add_definitions(-std=c++11 -Wall)

include_directories(
  .
)

#add_definitions(-D_DEBUG)


if(APPLE)
  find_library( OpenGL_LIBRARY OpenGL )
  set(EXTRA_LIBS ${OpenGL_LIBRARY})
endif(APPLE)

link_libraries(
  glfw3 GLEW ${EXTRA_LIBS} assimp
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

