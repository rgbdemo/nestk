#
# Try to find Freenect library and include path.
# Once done this will define
#
# Freenect_FOUND
# FREENECT_INCLUDE_DIR
# FREENECT_LIBRARIES
# 

IF (WIN32)
    FIND_PATH( FREENECT_INCLUDE_DIR libfreenect.h
        HINTS ${NESTK_ROOT_DIRS_HINTS} $ENV{PROGRAMFILES}/Freenect ${PROJECT_SOURCE_DIR}/src/nvgl/Freenect
        PATH_SUFFIXES include include/libfreenect libfreenect
        DOC "The directory where libfreenect.h resides")
    FIND_LIBRARY( FREENECT_LIBRARIES
        NAMES freenect
		PATHS
			${NESTK_ROOT_DIRS_HINTS}
			$ENV{PROGRAMFILES}/libfreenect/lib
		PATHS_SUFFIXES lib
		DOC "The freenect library")
ELSE (WIN32)
    FIND_PATH( FREENECT_INCLUDE_DIR libfreenect.h
        HINTS ${NESTK_ROOT_DIRS_HINTS}
        PATHS /usr /usr/local /sw /opt
        PATH_SUFFIXES include include/libfreenect libfreenect
        DOC "The directory where libfreenect.h resides")
    FIND_LIBRARY( FREENECT_LIBRARIES
        NAMES freenect
        HINTS ${NESTK_ROOT_DIRS_HINTS}
		PATHS
            /usr
            /usr/local
            /sw
            /opt/local
        PATHS_SUFFIXES lib lib64
        DOC "The freenect library")
ENDIF (WIN32)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FREENECT DEFAULT_MSG
    FREENECT_LIBRARIES FREENECT_INCLUDE_DIR)

mark_as_advanced(FREENECT_LIBRARIES FREENECT_INCLUDE_DIR)
if(FREENECT_FOUND)
  include_directories(${FREENECT_INCLUDE_DIRS})
  message(STATUS "Freenect found (include: ${FREENECT_INCLUDE_DIR}, lib: ${FREENECT_LIBRARIES})")
endif()

MARK_AS_ADVANCED( FREENECT_FOUND )
