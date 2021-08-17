cmake_minimum_required(VERSION 3.0.2)
project(tocabi_ecat)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if(${FORCE_REALROBOT} MATCHES ON)
  message(" forces realrobot compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
  set(COMPILE_REAL TRUE)
else()

ADD_DEFINITIONS(-DTIME_CHECK)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RES OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- kernel version : " ${UNAME_RES})

string(FIND ${UNAME_RES} "xenomai" wsloc)

if(${wsloc} EQUAL -1 )
  message(" simulation only compile mode ")
  ADD_DEFINITIONS(-DCOMPILE_SIMULATION)
  set(COMPILE_SIM TRUE)
else()
  message(" xenomai detected realrobot compilt mode ")
  ADD_DEFINITIONS(-DCOMPILE_REALROBOT)
  set(COMPILE_REAL TRUE)
endif() 

endif()

if(${COMPILE_REAL})

# -- xeno rtnet
execute_process(COMMAND xeno-config --posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND xeno-config --posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS} ${XENO_LDFLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS} ${XENO_LDFLAGS}")
# set(CMAKE_C_COMPILER gcc)
include_directories(/usr/xenomai/include /usr/xenomai/include/cobalt)
link_directories(/usr/xenomai/lib)
# -- xeno rtnet

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(tocabi_lib COMPONENTS)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES tocabi_ecat
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  /usr/local/include/soem
  ${catkin_INCLUDE_DIRS}
  ${tocabi_lib_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(tocabi_ecat_lib
  src/tocabi_ecat.cpp
  ${catkin_LIBRARIES}
)

add_library(tocabi_ecat_lower_lib
  src/tocabi_ecat_lower.cpp
  ${catkin_LIBRARIES}
)

add_library(tocabi_ecat_upper_lib
  src/tocabi_ecat_upper.cpp
  ${catkin_LIBRARIES}
)

add_executable(tocabi_ecat src/main.cpp)
add_executable(tocabi_ecat_lower src/ecat_lower.cpp)
add_executable(tocabi_ecat_upper src/ecat_upper.cpp)
add_executable(watcher src/watcher.cpp)
add_executable(watcher2 src/watcher_status.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tocabi_ecat
  tocabi_ecat_lib
  ${catkin_LIBRARIES}
  -lsoem
  -pthread
)

target_link_libraries(tocabi_ecat_lower
  tocabi_ecat_lower_lib
  ${catkin_LIBRARIES}
  -lsoem
  -pthread
)

target_link_libraries(tocabi_ecat_upper
  tocabi_ecat_upper_lib
  ${catkin_LIBRARIES}
  -lsoem
  -pthread
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

endif()