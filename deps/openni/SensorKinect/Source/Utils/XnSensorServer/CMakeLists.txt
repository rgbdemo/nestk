# Add search paths
# ----------------
include_directories(${OPENNI_INCLUDE_DIR}
					${XNCORE_INCLUDE_DIR}
					../../
)

# Get sources
# ------------
file(GLOB ${XNSENSORSERVER_TARGET_NAME}_sources ./*.cpp)

# Add the library
# ---------------
add_executable(
	${XNSENSORSERVER_TARGET_NAME}
	${${XNSENSORSERVER_TARGET_NAME}_sources}
)

target_link_libraries(
	${XNSENSORSERVER_TARGET_NAME}
	${XNSENSORSERVER_LIB_NAME}
	${XNDDK_LIB_NAME}
	${XNFORMAT_LIB_NAME}	
	${XNDEVICESENSOR_LIB_NAME}
	${XNCORE_LIB_NAME}
	${OPENNI_LIB_NAME}
	${LIBJPEG_LIB_NAME}
)

set_target_properties(
	${XNSENSORSERVER_TARGET_NAME}
	PROPERTIES
	INSTALL_RPATH "."
)

# Set install dir
# ---------------
install(
	TARGETS ${XNSENSORSERVER_TARGET_NAME}
	LIBRARY DESTINATION .
	RUNTIME DESTINATION .
)

