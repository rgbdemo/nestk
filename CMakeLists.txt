cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(nestk)
SET(NESTK_VERSION 0.7)
SET(NESTK_EMBEDDED_USE_FILE "${nestk_BINARY_DIR}/UseEmbeddedNestk.cmake" CACHE INTERNAL "Nestk cmake file to be included by parent project")
SET(NESTK_DEPS_DIR "" CACHE PATH "Path to nestk deps")

## Speficies default possible locations for external dependencies
SET(NESTK_ROOT_DIRS_HINTS ${NESTK_DEPS_DIR} ${NESTK_ROOT_DIRS_HINTS} ${CMAKE_INSTALL_PREFIX}
                          "C:/nestk" "$ENV{PROGRAMFILES}/nestk"
                          "$ENV{PROGRAMFILES}/PCL"
                          "$ENV{PROGRAMFILES}/boost/boost_1_46_1"
                          "$ENV{PROGRAMFILES}/flann 1.6.9"
			  "$ENV{PROGRAMFILES}/Eigen 3.0.0"
                          "$ENV{HOME}/nestk" "$ENV{HOME}/opt/nestk"
                          /usr/local/nestk /opt/nestk /usr/local /opt)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${nestk_SOURCE_DIR}/cmake)
INCLUDE("${nestk_SOURCE_DIR}/deps/cmake/utils.cmake")

IF (NOT EXECUTABLE_OUTPUT_PATH)
  SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
ENDIF()

## Default build mode is RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

################################################################################
### options
################################################################################

OPTION(NESTK_USE_GSL "Build GSL support." ${NESTK_USE_GSL})
OPTION(NESTK_USE_FREENECT "Build libfreenect support." ${NESTK_USE_FREENECT})
OPTION(NESTK_USE_OPENNI "Build OpenNI support." ${NESTK_USE_OPENNI})
OPTION(NESTK_USE_PCL "Build PCL support." ${NESTK_USE_PCL})
OPTION(NESTK_USE_PMDSDK "Build PMD/SDK supoprt." ${NESTK_USE_PMDSDK})
OPTION(NESTK_USE_ROS "Use inside ROS." ${NESTK_USE_ROS})

OPTION(NESTK_USE_EMBEDDED_FREENECT "Use embedded libfreenect." 1)
IF (UNIX AND NOT NESTK_USE_ROS)
    SET(DEFAULT_NESTK_USE_EMBEDDED_OPENNI 1)
ENDIF()
OPTION(NESTK_USE_EMBEDDED_OPENNI "Use embedded openni." ${DEFAULT_NESTK_USE_EMBEDDED_OPENNI})
OPTION(NESTK_USE_EMBEDDED_EIGEN "Use embedded eigen" 1)
OPTION(NESTK_USE_EMBEDDED_GLEW "Use embedded glew." 1)
OPTION(NESTK_USE_EMBEDDED_PMDSDK "Use embedded pmdsdk." 1)

OPTION(NESTK_BUILD_OBJECT_DETECTION "Build Anytime Detection System" ${NESTK_BUILD_OBJECT_DETECTION})

################################################################################
### Process embedded dependencies
################################################################################

ADD_SUBDIRECTORY(deps)

################################################################################
### Load default flags
################################################################################
CONFIGURE_FILE(UseEmbeddedNestk.cmake.in
               "${nestk_BINARY_DIR}/UseEmbeddedNestk.cmake" @ONLY)
INCLUDE("${nestk_BINARY_DIR}/UseEmbeddedNestk.cmake")

################################################################################
### nestk
################################################################################
INCLUDE_DIRECTORIES("${nestk_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${nestk_BINARY_DIR}")
ADD_SUBDIRECTORY( ntk )

################################################################################
### tests
################################################################################
OPTION(NESTK_BUILD_TESTS "Build Nestk tests" 0)
IF (NESTK_BUILD_TESTS)
  ADD_SUBDIRECTORY(tests)
ENDIF ()

################################################################################
### samples
################################################################################
OPTION(NESTK_BUILD_SAMPLES "Build Nestk samples" 1)
IF (NESTK_BUILD_SAMPLES)
  ADD_SUBDIRECTORY(samples)
ENDIF ()

################################################################################
### cmake files management
################################################################################

CONFIGURE_FILE(UseNestk.cmake.in
               ${nestk_BINARY_DIR}/UseNestk.cmake @ONLY)

FILE(GLOB CMAKE_FILES "cmake/*.cmake")

INSTALL(FILES
            "${nestk_BINARY_DIR}/UseNestk.cmake"
            ${CMAKE_FILES}
        DESTINATION share/cmake)
