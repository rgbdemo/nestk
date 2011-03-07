# Add search paths
# ----------------
include_directories(${OPENNI_INCLUDE_DIR}
					${XNCORE_INCLUDE_DIR}
					${LIBJPEG_INCLUDE_DIR}
					../
)

# Get sources
# ------------
file(GLOB ${XNDEVICESENSOR_LIB_NAME}_sources ./*.cpp)

# Add the library
# ---------------
add_library(
	${XNDEVICESENSOR_LIB_NAME} 
	SHARED 
	${${XNDEVICESENSOR_LIB_NAME}_sources}
)

target_link_libraries(
        ${XNDEVICESENSOR_LIB_NAME}
        ${XNFORMAT_LIB_NAME}
        ${XNDDK_LIB_NAME}
)


# Set install dir
# ---------------
install(
	TARGETS ${XNDEVICESENSOR_LIB_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

