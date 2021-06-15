cmake_minimum_required(VERSION 3.0.2)
project(tocabi_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -Ofast)

## if -dVE-march=native  -mavx -mfma

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  eigen_conversions
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tocabi_ecat
  tocabi_lib
  tocabi_description
  tocabi_msgs
  tocabi_cc
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")
#FIND_PACKAGE(Eigen3 REQUIRED)
#FIND_PACKAGE(RBDL COMPONENTS URDFREADER REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES tocabi_controller
   CATKIN_DEPENDS roscpp roslib tocabi_lib tocabi_cc tocabi_msgs tocabi_ecat
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(${PROJECT_NAME}_lib
  src/state_manager.cpp
  src/tocabi_controller.cpp
)

#add_library(wbc_lib
#  src/wholebody_controller.cpp
#)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/main.cpp)
#add_executable(rbdl_test src/simple_example.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

#target_link_libraries(rbdl_test
#  ${catkin_LIBRARIES}
#  -lrbdl
#  -lrbdl_urdfreader
#  )


target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  tocabi_lib
  tocabi_cc
  -lpthread
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)


add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})