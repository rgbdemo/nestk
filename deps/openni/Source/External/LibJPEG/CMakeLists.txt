#project(LibJPEG)

file(GLOB ${JPEG_LIB_NAME}_sources *.c)
include_directories(${OPENNI_INCLUDE_DIR})
add_library(${JPEG_LIB_NAME} STATIC ${${JPEG_LIB_NAME}_sources})
target_link_libraries(${JPEG_LIB_NAME} ${OPENNI_LIB_NAME})
