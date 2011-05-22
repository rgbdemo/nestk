###############################################################################
# Find Nestk
#
# This sets the following variables:
# NESTK_FOUND - True if FLANN was found.
# NESTK_INCLUDE_DIRS - Directories containing the FLANN include files.
# NESTK_LIBRARIES - Libraries needed to use FLANN.

SET(NESTK_ROOT_DIRS_HINTS ${NESTK_ROOT_DIRS_HINTS} 
    $ENV{HOME}/nestk 
    $ENV{HOME}/opt/nestk
    /opt/nestk /usr/local/nestk
)

find_path(NESTK_INCLUDE_DIR ntk/core.h
          HINTS ${NESTK_ROOT_DIRS_HINTS} "${NESTK_ROOT}" "$ENV{NESTK_ROOT}"
          PATH_SUFFIXES include)

find_path(NESTK_CMAKE_DIR UseNestk.cmake
          HINTS ${NESTK_ROOT_DIRS_HINTS} "${NESTK_ROOT}" "$ENV{NESTK_ROOT}"
          PATH_SUFFIXES share/cmake)

find_library(NESTK_LIBRARIES
          NAMES nestk
	  HINTS ${NESTK_ROOT_DIRS_HINTS} "${NESTK_ROOT}" "$ENV{NESTK_ROOT}"
	  PATH_SUFFIXES lib)

IF (NESTK_INCLUDE_DIR)
	SET(NESTK_FOUND 1 CACHE STRING "Set to 1 if NESTK is found, 0 otherwise")
ELSE ()
	SET( NESTK_FOUND 0 CACHE STRING "Set to 1 if NESTK is found, 0 otherwise")
ENDIF ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Nestk DEFAULT_MSG
    NESTK_LIBRARIES NESTK_INCLUDE_DIR NESTK_CMAKE_DIR)

SET(NESTK_USE_FILE ${NESTK_CMAKE_DIR}/UseNestk.cmake)

MARK_AS_ADVANCED( NESTK_FOUND )
