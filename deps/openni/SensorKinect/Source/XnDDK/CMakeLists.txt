# Add search paths
# ----------------
include_directories(${OPENNI_INCLUDE_DIR}
					${XNCORE_INCLUDE_DIR}
					../
)

# Get sources
# ------------
file(GLOB ${XNDDK_LIB_NAME}_sources ./*.cpp)

# Add the library
# ---------------
add_library(
	${XNDDK_LIB_NAME} 
	SHARED 
	${${XNDDK_LIB_NAME}_sources}
)

target_link_libraries(
	${XNDDK_LIB_NAME}
	${XNCORE_LIB_NAME}
	${NIMCODECS_LIB_NAME}
	${LIBJPEG_LIB_NAME}
	${XNFORMAT_LIB_NAME}
)
# Set install dir
# ---------------
install(
	TARGETS ${XNDDK_LIB_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

