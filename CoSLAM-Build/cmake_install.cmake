# Install script for directory: /home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM

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
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/CoSLAM-Build/CoSLAM")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/CoSLAM")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/share/CoSLAM/klt_tracker.cg;/usr/share/CoSLAM/klt_tracker_with_gain.cg;/usr/share/CoSLAM/pyramid_pass2.cg;/usr/share/CoSLAM/klt_detector_traverse_histpyr.cg;/usr/share/CoSLAM/klt_detector_nonmax.cg;/usr/share/CoSLAM/pyramid_with_derivative_pass2.cg;/usr/share/CoSLAM/pyramid_with_derivative_pass1h.cg;/usr/share/CoSLAM/undistort_parametric.cg;/usr/share/CoSLAM/klt_detector_discriminator.cg;/usr/share/CoSLAM/pyramid_pass1h.cg;/usr/share/CoSLAM/klt_detector_build_histpyr.cg;/usr/share/CoSLAM/flow_warp_image.cg;/usr/share/CoSLAM/klt_detector_pass2.cg;/usr/share/CoSLAM/klt_detector_pass1.cg;/usr/share/CoSLAM/pyramid_with_derivative_pass1v.cg;/usr/share/CoSLAM/pyramid_pass1v.cg")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/share/CoSLAM" TYPE FILE FILES
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_tracker.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_tracker_with_gain.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/pyramid_pass2.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_detector_traverse_histpyr.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_detector_nonmax.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/pyramid_with_derivative_pass2.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/pyramid_with_derivative_pass1h.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/undistort_parametric.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_detector_discriminator.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/pyramid_pass1h.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_detector_build_histpyr.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/flow_warp_image.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_detector_pass2.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/klt_detector_pass1.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/pyramid_with_derivative_pass1v.cg"
    "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/src/tracking/CGKLT/Shaders/pyramid_pass1v.cg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/CoSLAM-Build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/robot/CoSLAMUpdate/OFFICIAL/CoSLAMWen/CoSLAM/CoSLAM-Build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
