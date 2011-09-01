#
# Root nestk
#
#   nicolas@burrus.name

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

PROJECT(nestk_deps)
SET(NESTK_DEPS_VERSION 0.1)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${nestk_deps_SOURCE_DIR}/cmake)

INCLUDE( ${nestk_deps_SOURCE_DIR}/cmake/utils.cmake )
INCLUDE( ${nestk_deps_SOURCE_DIR}/cmake/msvc.cmake )

IF(NOT NESTK_USE_ROS)

## GLEW
IF(NESTK_USE_EMBEDDED_GLEW)
  ## GLEW
  set(GLEW_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/glew/include" CACHE FILEPATH "" FORCE)
  set(GLEW_LIBRARIES "glew" CACHE FILEPATH "" FORCE)
  ADD_SUBDIRECTORY("${nestk_deps_SOURCE_DIR}/glew")
ENDIF()

## FREENECT and OPENNI deps
IF((NESTK_USE_FREENECT AND NESTK_USE_EMBEDDED_FREENECT) OR (NESTK_USE_OPENNI AND NESTK_USE_EMBEDDED_OPENNI))
  ## LIBUSB
  IF (NOT LIBUSB_1_INCLUDE_DIR)
    if(WIN32)
      set(LIBUSB_1_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/libusb-1.0/include" CACHE FILEPATH "" FORCE)
      IF (CMAKE_COMPILER_IS_GNUCXX)
        set(LIBUSB_1_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/libusb-1.0/lib/gcc/libusb.a" CACHE FILEPATH "" FORCE)
      ELSE()
        set(LIBUSB_1_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/libusb-1.0/lib/msvc/libusb.lib" CACHE FILEPATH "" FORCE)
      ENDIF()
    ELSEIF(APPLE)
      IF (NOT LIBUSB_1_INCLUDE_DIR)
        set(LIBUSB_1_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/macosx/libusb-7da756e" CACHE FILEPATH "" FORCE)
        set(LIBUSB_1_LIBRARY "usb" CACHE FILEPATH "" FORCE)
        set(USE_EMBEDDED_LIBUSB 1 CACHE INTERNAL "")
      ENDIF()
    ENDIF()
  ENDIF()
  IF(APPLE AND USE_EMBEDDED_LIBUSB)
    ADD_SUBDIRECTORY("${nestk_deps_SOURCE_DIR}/macosx/libusb-7da756e")
  ENDIF()
  INCLUDE_DIRECTORIES(${LIBUSB_1_INCLUDE_DIR})
  FIND_PACKAGE(libusb-1.0 REQUIRED)
ENDIF()

## GLUT
if(WIN32)
  IF (NOT GLUT_INCLUDE_DIR)
    IF (CMAKE_COMPILER_IS_GNUCXX)
      set(GLUT_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/freeglut/include" CACHE FILEPATH "" FORCE)
      set(GLUT_glut_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/freeglut/lib/libfreeglut.a" CACHE FILEPATH "" FORCE)
    ELSE()
      set(GLUT_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/glut-msvc" CACHE FILEPATH "" FORCE)
      set(GLUT_glut_LIBRARY "${nestk_deps_SOURCE_DIR}/win32/glut-msvc/glut32.lib" CACHE FILEPATH "" FORCE)
    ENDIF()
  ENDIF()
  INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
endif()

## FREENECT
IF(NESTK_USE_FREENECT AND NESTK_USE_EMBEDDED_FREENECT)
    ## PTHREADS
    if(WIN32)
      IF (NOT THREADS_PTHREADS_INCLUDE_DIR)
        set(THREADS_PTHREADS_INCLUDE_DIR "${nestk_deps_SOURCE_DIR}/win32/pthreads/include" CACHE FILEPATH "" FORCE)
        set(THREADS_USE_PTHREADS_WIN32 true)
        FIND_PACKAGE(Threads)
        set(PTHREAD_LIBRARY ${CMAKE_THREAD_LIBS_INIT} CACHE FILEPATH "" FORCE)
      ENDIF()
      include_directories("${nestk_deps_SOURCE_DIR}/libfreenect/platform/windows")
      include_directories("${nestk_deps_SOURCE_DIR}/libfreenect/platform/windows/libusb10emu")
    else()
      set(PTHREAD_LIBRARY pthread)
    endif()
    ADD_SUBDIRECTORY(libfreenect)
ENDIF()

## OPENNI
IF (NESTK_USE_OPENNI AND NESTK_USE_EMBEDDED_OPENNI AND NOT WIN32)
    set(USB_INCLUDE_DIR ${LIBUSB_1_INCLUDE_DIR})
    set(USB_LIBRARIES ${LIBUSB_1_LIBRARY})
    ADD_SUBDIRECTORY(openni)
ENDIF()

## EIGEN
IF (NESTK_USE_EMBEDDED_EIGEN)
  SET(eigen3_INCLUDE_DIRS ${nestk_deps_SOURCE_DIR}/eigen)
  SET(EIGEN_INCLUDE_DIR ${eigen3_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${eigen3_INCLUDE_DIRS})
  ADD_SUBDIRECTORY(eigen)
ENDIF()

ENDIF(NOT NESTK_USE_ROS)

################################################################################
# cmake scripts
################################################################################
ADD_SUBDIRECTORY(cmake)
