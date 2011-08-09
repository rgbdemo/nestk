#project(nimCodecs)

file(GLOB ${NIMCODECS_LIB_NAME}_sources ./*.cpp)

include_directories(${JPEG_INCLUDE_DIR})
include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${OPENNI_MODULES_DIR}/Common/)

add_library(${NIMCODECS_LIB_NAME} SHARED ${${NIMCODECS_LIB_NAME}_sources})
target_link_libraries(${NIMCODECS_LIB_NAME} ${JPEG_LIBRARY} ${OPENNI_LIB_NAME})

install(TARGETS ${NIMCODECS_LIB_NAME}
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
