# Install script for directory: /home/nonsense/Projects/FinalTask/Gazebo/catkin_ws/src/RobotBase/hector_quadrotor_noetic/takeoff_land

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nonsense/Projects/FinalTask/Gazebo/catkin_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nonsense/Projects/FinalTask/Gazebo/catkin_ws/build/RobotBase/hector_quadrotor_noetic/takeoff_land/catkin_generated/installspace/takeoff_land.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/takeoff_land/cmake" TYPE FILE FILES
    "/home/nonsense/Projects/FinalTask/Gazebo/catkin_ws/build/RobotBase/hector_quadrotor_noetic/takeoff_land/catkin_generated/installspace/takeoff_landConfig.cmake"
    "/home/nonsense/Projects/FinalTask/Gazebo/catkin_ws/build/RobotBase/hector_quadrotor_noetic/takeoff_land/catkin_generated/installspace/takeoff_landConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/takeoff_land" TYPE FILE FILES "/home/nonsense/Projects/FinalTask/Gazebo/catkin_ws/src/RobotBase/hector_quadrotor_noetic/takeoff_land/package.xml")
endif()

