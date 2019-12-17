cmake_minimum_required(VERSION 2.8.3)
project(metric_optimization)

add_compile_options(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(ifopt)
find_package(RBDL REQUIRED)
find_package(coinhsl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  
  roscpp 
  rospy  
  std_msgs  
  DEPENDS 
)
add_subdirectory($ENV{HOME}/git/dlib dlib_build)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${RBDL_INCLUDE_DIRS}  
  ${DLIB_INCLUDE_DIRECTORY}
)

add_library(${PROJECT_NAME}_lib
  src/${PROJECT_NAME}/main.cpp
  src/${PROJECT_NAME}/optimization/problem.cpp
)

add_library(${PROJECT_NAME}_penalty_lib
  src/${PROJECT_NAME}/test.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ifopt::ifopt_ipopt
  ipopt
  rbdl
  # coinasl
  # coinblas
  # coinlapack
  # coinmetis
  coinhsl
)

target_link_libraries(${PROJECT_NAME}_penalty_lib
  ${catkin_LIBRARIES}
  ${DLIB_LIBRARIES}
  dlib::dlib
  rbdl
)
add_executable(${PROJECT_NAME}_ipopt src/${PROJECT_NAME}/main.cpp)
add_executable(${PROJECT_NAME}_penalty src/${PROJECT_NAME}/test.cpp)

target_link_libraries(${PROJECT_NAME}_ipopt
  ${PROJECT_NAME}_lib
)

target_link_libraries(${PROJECT_NAME}_penalty
  ${PROJECT_NAME}_penalty_lib
  dlib::dlib
)
# http://www.hsl.rl.ac.uk/download/coinhsl-archive/2014.01.17/
# https://projects.coin-or.org/Ipopt
