cmake_minimum_required(VERSION 3.0.2)
project(tocabi_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-O)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  eigen_conversions
  sensor_msgs
  std_msgs
  mujoco_ros_msgs
  tf2
  tf2_ros
  tocabi_lib
  tocabi_description
  tocabi_msgs
)

find_package(tocabi_cc QUIET)
find_package(tocabi_avatar QUIET)

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
   CATKIN_DEPENDS roscpp roslib tocabi_lib tocabi_msgs tocabi_cc tocabi_avatar
#  DEPENDS system_lib
)


execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)

string(FIND ${UNAME_RES} "xenomai" wsloc)

if(${wsloc} EQUAL -1 )
  message(" Controller : simulation only compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_SIMULATION)
else()
  message(" xenomai detected realrobot compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
endif() 


IF(tocabi_cc_FOUND)
 message(" -- tocabi_cc found" )
   ADD_DEFINITIONS(-DCOMPILE_TOCABI_CC)
ENDIF()

IF(tocabi_avatar_FOUND)
 message(" -- tocabi_avatar found" )
   ADD_DEFINITIONS(-DCOMPILE_TOCABI_AVATAR)
ENDIF()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tocabi_cc_INCLUDE_DIRS}
  ${tocabi_avatar_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/state_manager.cpp
  src/tocabi_controller.cpp
)

link_directories(${CATKIN_DEVEL_PREFIX}/lib})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/main.cpp)
add_executable(shutdown src/shutdown.cpp)

add_executable(shm_reset src/shm_manager.cpp)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${tocabi_cc_LIBRARIES}
  ${tocabi_avatar_LIBRARIES}
  -lpthread
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)

add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(shm_reset ${catkin_EXPORTED_TARGETS})