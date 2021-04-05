cmake_minimum_required(VERSION 3.0.2)
project(tocabi_ecat_lower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

##set(CMAKE_BUILD_TYPE RELWITHDEBINFO)

find_package(catkin REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES tocabi_ecat
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/local/include/soem
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(tocabi_ecat_lower_lib
  src/tocabi_ecat_lower.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  tocabi_ecat_lib
  ${catkin_LIBRARIES}
  -lsoem
  -pthread
)
