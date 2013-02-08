###############################################################################
# Find SoftKinetic SDK
#
# This sets the following variables:
# SOFTKINETIC_FOUND - True if SOFTKINETIC was found.
# SOFTKINETIC_INCLUDE_DIR - Directories containing the SOFTKINETIC include files.
# SOFTKINETIC_LIBRARY - Libraries needed to use SOFTKINETIC.

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROGRAM_FILES_PATHS "$ENV{PROGRAMW6432}/SoftKinetic/DepthSenseSDK")
else ()
    set(PROGRAM_FILES_PATHS "$ENV{PROGRAMFILES}/SoftKinetic/DepthSenseSDK")
endif()

find_path(SOFTKINETIC_INCLUDE_DIR DepthSense.hxx
          HINTS ${NESTK_ROOT_DIRS_HINTS} "${SOFTKINETIC_ROOT}" "$ENV{SOFTKINETIC_ROOT}"
          PATHS "${PROGRAM_FILES_PATHS}"
          PATH_SUFFIXES include)

find_library(SOFTKINETIC_LIBRARY
           NAMES DepthSense
           HINTS ${NESTK_ROOT_DIRS_HINTS} "${SOFTKINETIC_ROOT}" "$ENV{SOFTKINETIC_ROOT}"
           PATHS "${PROGRAM_FILES_PATHS}"
           PATH_SUFFIXES lib )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SOFTKINETIC DEFAULT_MSG SOFTKINETIC_LIBRARY SOFTKINETIC_INCLUDE_DIR)
mark_as_advanced(SOFTKINETIC_LIBRARY SOFTKINETIC_INCLUDE_DIR)

if(SOFTKINETIC_FOUND)
    message(STATUS "SOFTKINETIC found (include: ${SOFTKINETIC_INCLUDE_DIR}, lib: ${SOFTKINETIC_LIBRARY})")
endif(SOFTKINETIC_FOUND)
