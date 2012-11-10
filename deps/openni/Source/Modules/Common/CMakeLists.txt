#project(nimCodecs)

file(GLOB ${NIMCOMMON_LIB_NAME}_sources ./*.cpp)

include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${OPENNI_MODULES_DIR}/Common/)

add_library(${NIMCOMMON_LIB_NAME} SHARED ${${NIMCOMMON_LIB_NAME}_sources})
target_link_libraries(${NIMCOMMON_LIB_NAME} ${OPENNI_LIB_NAME})

install(TARGETS ${NIMCOMMON_LIB_NAME}
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

