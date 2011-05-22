#
# Try to find GLEW library and include path.
# Once done this will define
#
# GLEW_FOUND
# GLEW_INCLUDE_DIR
# GLEW_LIBRARIES
# 

IF (WIN32)
    FIND_PATH( GLEW_INCLUDE_DIR GL/glew.h
        HINTS ${NESTK_ROOT_DIRS_HINTS} $ENV{PROGRAMFILES}/GLEW ${PROJECT_SOURCE_DIR}/src/nvgl/glew
        PATH_SUFFIXES include
		DOC "The directory where GL/glew.h resides")
	FIND_LIBRARY( GLEW_LIBRARIES
		NAMES glew GLEW glew32 glew32s
		PATHS
            $ENV{PROGRAMFILES}/GLEW/lib
            ${PROJECT_SOURCE_DIR}/src/nvgl/glew/bin
            ${PROJECT_SOURCE_DIR}/src/nvgl/glew/lib
        PATH_SUFFIXES lib
		DOC "The GLEW library")
ELSE (WIN32)
    FIND_PATH( GLEW_INCLUDE_DIR GL/glew.h
        HINTS ${NESTK_ROOT_DIRS_HINTS}
        PATHS /usr /usr/local /sw /opt
        PATH_SUFFIXES include
		DOC "The directory where GL/glew.h resides")
    FIND_LIBRARY( GLEW_LIBRARIES
		NAMES GLEW glew
        HINTS ${NESTK_ROOT_DIRS_HINTS}
		PATHS
            /usr
            /usr/local
            /sw
            /opt/local
        PATHS_SUFFIXES lib lib64
		DOC "The GLEW library")
ENDIF (WIN32)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLEW DEFAULT_MSG
    GLEW_LIBRARIES GLEW_INCLUDE_DIR)

mark_as_advanced(GLEW_LIBRARIES GLEW_INCLUDE_DIR)
if(GLEW_FOUND)
  include_directories(${GLEW_INCLUDE_DIRS})
  message(STATUS "GLEW found (include: ${GLEW_INCLUDE_DIR}, lib: ${GLEW_LIBRARIES})")
endif()


IF (GLEW_INCLUDE_DIR)
	SET( GLEW_FOUND 1 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ELSE (GLEW_INCLUDE_DIR)
	SET( GLEW_FOUND 0 CACHE STRING "Set to 1 if GLEW is found, 0 otherwise")
ENDIF (GLEW_INCLUDE_DIR)

MARK_AS_ADVANCED( GLEW_FOUND )
