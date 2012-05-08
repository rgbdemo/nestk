cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(nestk)
SET(NESTK_VERSION 0.7)

################################################################################
### extra flags
################################################################################
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(DEFAULT_NESTK_EXTRA_CMAKE_CXX_FLAGS "-Wall -W -Wno-unused -Wno-sign-compare")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

SET(NESTK_EXTRA_CMAKE_CXX_FLAGS  ${DEFAULT_NESTK_EXTRA_CMAKE_CXX_FLAGS}
    CACHE STRING "Extra flags appended to CMAKE_CXX_FLAGS" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NESTK_EXTRA_CMAKE_CXX_FLAGS}" )

OPTION(NESTK_BUILD_OBJECT_DETECTION "Build nestk object detection module." 0)

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
