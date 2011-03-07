#project(nimRecorder)

file(GLOB ${NIMRECORDER_LIB_NAME}_sources ./*.cpp)

include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${OPENNI_MODULES_DIR}/Common/)


add_library(${NIMRECORDER_LIB_NAME} SHARED ${${NIMRECORDER_LIB_NAME}_sources})
target_link_libraries(${NIMRECORDER_LIB_NAME} ${OPENNI_LIB_NAME} ${NIMCOMMON_LIB_NAME})


install(TARGETS ${NIMRECORDER_LIB_NAME}
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
