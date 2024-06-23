# CMake 프로그램의 최소 버전
cmake_minimum_required(VERSION 3.11)

# 프로젝트 정보
project(
  ModooCode
  VERSION 0.1
  DESCRIPTION "예제 프로젝트"
  LANGUAGES CXX)


# add_library(shape STATIC ${SRC_FILES})
set (SOURCES
  main.cpp
  source/controller.cpp
  source/filter.cpp
  source/kinematics.cpp
  source/trajectory.cpp
)

# set(CMAKE_INSTALL_RPATH $ORIGIN)

# find_library(CASADI_LIBRARY
#     NAMES casadi
#     PATHS /home/jinsong/casadi/build/lib
#     )
# IF (NOT CASADI_LIBRARY)
#   MESSAGE(WARNING "Could not find Eigen3 on your system. Please install it!")
# ENDIF (NOT CASADI_LIBRARY)


add_executable (main ${SOURCES}) #change YCM


target_include_directories(main PUBLIC
  ${CMAKE_SOURCE_DIR}/data
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/../../include
  #${CMAKE_SOURCE_DIR}/../../include/mujoco
  /usr/include/eigen3/
  )



# link_directories(main          #Library directory
#     # ${CMAKE_SOURCE_DIR}/../../lib
#     /home/jinsong/casadi/build/lib

# )

# set_target_properties(main PROPERTIES
#     LINK_FLAGS "-Wl,-rpath,/home/jinsong/casadi/build/lib"
# )

target_link_libraries(main
    mujoco
    glfw
    dl

)
