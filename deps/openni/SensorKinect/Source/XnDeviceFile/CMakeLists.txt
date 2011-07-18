# Add search paths
# ----------------
include_directories(${OPENNI_INCLUDE_DIR}
					${XNCORE_INCLUDE_DIR}
					../
)

# Get sources
# ------------
file(GLOB ${XNDEVICEFILE_LIB_NAME}_sources ./*.cpp)

# Add the library
# ---------------
add_library(
	${XNDEVICEFILE_LIB_NAME} 
	SHARED 
	${${XNDEVICEFILE_LIB_NAME}_sources}
)
target_link_libraries(
	${XNDEVICEFILE_LIB_NAME}
	${XNFORMAT_LIB_NAME}
	${XNDDK_LIB_NAME}

)
# Set install dir
# ---------------
install(
	TARGETS ${XNDEVICEFILE_LIB_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

