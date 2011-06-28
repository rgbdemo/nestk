################################################################################
### extra flagssrc
################################################################################
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(DEFAULT_NESTK_EXTRA_CMAKE_CXX_FLAGS "-fPIC -Wall -W -Wno-unused -Wno-sign-compare")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

SET(NESTK_EXTRA_CMAKE_CXX_FLAGS  ${DEFAULT_NESTK_EXTRA_CMAKE_CXX_FLAGS}
    CACHE STRING "Extra flags appended to CMAKE_CXX_FLAGS" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${NESTK_EXTRA_CMAKE_CXX_FLAGS}" )

## Check if private part is enabled.
IF(IS_DIRECTORY "@nestk_SOURCE_DIR@/ntk/private")
  ADD_DEFINITIONS(-DNESTK_PRIVATE)
  SET(HAVE_NESTK_PRIVATE 1 CACHE INTERNAL "Nestk private features are available")
ENDIF()

################################################################################
### Find dependencies
################################################################################
# Look for external dependencies
IF (NOT NESTK_USE_ROS)
  INCLUDE(${nestk_SOURCE_DIR}/cmake/find_nestk_deps.cmake)
  # Load parameters for embedded dependencies
  INCLUDE(${nestk_deps_BINARY_DIR}/cmake/UseNestkDeps.cmake)
ENDIF()

IF (NESTK_USE_ROS)
  INCLUDE(${nestk_deps_BINARY_DIR}/cmake/UseNestkDepsROS.cmake)
  INCLUDE_DIRECTORIES(${nestk_SOURCE_DIR})
ENDIF()

INCLUDE_DIRECTORIES("@nestk_SOURCE_DIR@")
INCLUDE_DIRECTORIES("@nestk_BINARY_DIR@")
LINK_DIRECTORIES("@LIBRARY_OUTPUT_PATH@")
