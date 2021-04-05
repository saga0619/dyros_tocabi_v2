cmake_minimum_required(VERSION 3.0.2)
project(tocabi_ecat_upper)

##set(CMAKE_BUILD_TYPE RELWITHDEBINFO)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES tocabi_ecat
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  /usr/local/include/soem
)

add_library(tocabi_ecat_upper_lib
  src/tocabi_ecat_upper.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  tocabi_ecat_lib
  ${catkin_LIBRARIES}
  -lsoem
  -pthread
)
