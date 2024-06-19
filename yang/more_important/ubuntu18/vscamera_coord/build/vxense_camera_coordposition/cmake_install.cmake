# Install script for directory: /home/yang/Vxense/AGV_use_ws/yang/more_important/ubuntu18/vscamera_coord/src/vxense_camera_coordposition

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yang/Vxense/AGV_use_ws/yang/more_important/ubuntu18/vscamera_coord/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yang/Vxense/AGV_use_ws/yang/more_important/ubuntu18/vscamera_coord/build/vxense_camera_coordposition/catkin_generated/installspace/vxense_camera_coordposition.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vxense_camera_coordposition/cmake" TYPE FILE FILES
    "/home/yang/Vxense/AGV_use_ws/yang/more_important/ubuntu18/vscamera_coord/build/vxense_camera_coordposition/catkin_generated/installspace/vxense_camera_coordpositionConfig.cmake"
    "/home/yang/Vxense/AGV_use_ws/yang/more_important/ubuntu18/vscamera_coord/build/vxense_camera_coordposition/catkin_generated/installspace/vxense_camera_coordpositionConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vxense_camera_coordposition" TYPE FILE FILES "/home/yang/Vxense/AGV_use_ws/yang/more_important/ubuntu18/vscamera_coord/src/vxense_camera_coordposition/package.xml")
endif()

