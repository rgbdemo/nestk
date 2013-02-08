###############################################################################
# Find SoftKinetic IISU SDK
#
# This sets the following variables:
# SOFTKINETIC_IISU_FOUND - True if SOFTKINETIC_IISU was found.
# SOFTKINETIC_IISU_INCLUDE_DIR - Directories containing the SOFTKINETIC_IISU include files.
# SOFTKINETIC_IISU_LIBRARY - Libraries needed to use SOFTKINETIC_IISU.

find_path(SOFTKINETIC_IISU_INCLUDE_DIR SDK/iisuSDK.h
          HINTS ${NESTK_ROOT_DIRS_HINTS} "${SOFTKINETIC_IISU_ROOT}" "$ENV{SOFTKINETIC_IISU_ROOT}"
          PATHS "C:/Softkinetic/iisu" "$ENV{PROGRAMW6432}/SoftKinetic/DepthSenseSDK" "$ENV{PROGRAMFILES}/SoftKinetic/DepthSenseSDK"
          PATH_SUFFIXES include/iisu)

find_library(SOFTKINETIC_IISU_LIBRARY
           NAMES iisuSDK
           HINTS ${NESTK_ROOT_DIRS_HINTS} "${SOFTKINETIC_IISU_ROOT}" "$ENV{SOFTKINETIC_IISU_ROOT}"
           PATHS "C:/Softkinetic/iisu" "$ENV{PROGRAMW6432}/SoftKinetic/DepthSenseSDK" "$ENV{PROGRAMFILES}/SoftKinetic/DepthSenseSDK"
           PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SOFTKINETIC_IISU DEFAULT_MSG SOFTKINETIC_IISU_LIBRARY SOFTKINETIC_IISU_INCLUDE_DIR)
mark_as_advanced(SOFTKINETIC_IISU_LIBRARY SOFTKINETIC_IISU_INCLUDE_DIR)

if(SOFTKINETIC_IISU_FOUND)
    message(STATUS "SOFTKINETIC_IISU found (include: ${SOFTKINETIC_IISU_INCLUDE_DIR}, lib: ${SOFTKINETIC_IISU_LIBRARY})")
endif(SOFTKINETIC_IISU_FOUND)
