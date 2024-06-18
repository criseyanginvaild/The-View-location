cmake_minimum_required(VERSION 3.0.2)
project(zoom_factor_publisher)

set(PYTHON_EXECUTABLE /usr/bin/python)

## Compile as C++11, supported in ROS Kinetic and newer
##add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(X11 REQUIRED)
find_library(XI_LIBRARY Xi PATHS ${X11_LIB_SEARCH_PATH})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zoom_factor_publisher
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/touch.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${X11_LIBRARIES}
  ${XI_LIBRARY} # Add this line to explicitly link against libXi
)
