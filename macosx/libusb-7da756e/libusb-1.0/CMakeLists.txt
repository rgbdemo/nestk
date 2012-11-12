PROJECT(libusb)

SET(sources 
  libusbi.h
  config.h
  core.c
  descriptor.c
  io.c
  sync.c
  os/darwin_usb.c
  os/darwin_usb.h
  os/threads_posix.h
  os/poll_posix.h
)

SET_DIRECTORY_PROPERTIES(PROPERTIES
                         COMPILE_DEFINITIONS "NDEBUG")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
ADD_LIBRARY(usb SHARED ${sources})
TARGET_LINK_LIBRARIES(usb ${IOKIT_LIBRARY} ${COREFOUNDATION_LIBRARY})
set_target_properties ( usb PROPERTIES
  ARCHIVE_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}"
  RUNTIME_OUTPUT_DIRECTORY "${EXECUTABLE_OUTPUT_PATH}")
