add_subdirectory(./Common)
add_subdirectory(./nimCodecs)
add_subdirectory(./nimMockNodes)
add_subdirectory(./nimRecorder)

add_dependencies(${NIMCOMMON_LIB_NAME} ${OPENNI_LIB_NAME})
add_dependencies(${NIMCODECS_LIB_NAME} ${JPEG_LIB_NAME} ${OPENNI_LIB_NAME})
add_dependencies(${NIMMOCKNODES_LIB_NAME} ${OPENNI_LIB_NAME})
add_dependencies(${NIMRECORDER_LIB_NAME} ${OPENNI_LIB_NAME} ${NIMCOMMON_LIB_NAME})
