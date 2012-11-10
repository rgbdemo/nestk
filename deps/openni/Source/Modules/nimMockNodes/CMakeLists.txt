#project(nimMockNodes)

file(GLOB ${NIMMOCKNODES_LIB_NAME}_sources ./*.cpp)

include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${OPENNI_MODULES_DIR}/Common/)

add_library(${NIMMOCKNODES_LIB_NAME} SHARED ${${NIMMOCKNODES_LIB_NAME}_sources})
target_link_libraries(${NIMMOCKNODES_LIB_NAME} ${OPENNI_LIB_NAME})

install(TARGETS ${NIMMOCKNODES_LIB_NAME}
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
