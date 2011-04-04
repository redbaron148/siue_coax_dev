# Install script for directory: /home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/../deploy/linux")
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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/com" TYPE FILE FILES
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbversion.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbapi.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbchannel.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbconst.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbmessage.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbsimple.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbstate.h"
    "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/src/../include/com/sbcommloop.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/aaron/ros_pkgs/siue_coax_dev/include/coax-software/communication/build-linux/src/libsbcom.a")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

