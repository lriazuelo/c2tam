# Install script for directory: /home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libparser.a")
FILE(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/libparser.a")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/slam_parser/parser/FlexLexer.h;/usr/local/include/slam_parser/parser/scanner.h;/usr/local/include/slam_parser/parser/slam_context.h;/usr/local/include/slam_parser/parser/bison_parser.h;/usr/local/include/slam_parser/parser/driver.h;/usr/local/include/slam_parser/parser/commands.h;/usr/local/include/slam_parser/parser/stack.hh;/usr/local/include/slam_parser/parser/location.hh;/usr/local/include/slam_parser/parser/position.hh")
FILE(INSTALL DESTINATION "/usr/local/include/slam_parser/parser" TYPE FILE FILES
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/FlexLexer.h"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/scanner.h"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/slam_context.h"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/bison_parser.h"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/driver.h"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/commands.h"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/stack.hh"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/location.hh"
    "/home/luis/workspace/ros_stacks/ctam_stacks/ctam_4year/re_ctam/ctam_mapping/EXTERNAL/g2o/g2o/examples/interactive_slam/slam_parser/parser/position.hh"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

