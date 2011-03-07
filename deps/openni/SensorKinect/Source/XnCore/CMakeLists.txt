# Add search paths
# ----------------
include_directories(${OPENNI_INCLUDE_DIR})
include_directories(${XNCORE_INCLUDE_DIR})
include_directories(../)

# Get sources
# ------------
file(GLOB ${XNCORE_LIB_NAME}_sources ./*.cpp)

# Add the library
# ---------------
add_library(
	${XNCORE_LIB_NAME} 
	SHARED 
	${${XNCORE_LIB_NAME}_sources}
)

# Set link libraries
# -------------------
target_link_libraries(${XNCORE_LIB_NAME} ${OPENNI_LIB_NAME})
#target_link_libraries(${NIMMOCKNODES_LIB_NAME} ${OPENNI_LIB_NAME})

# Set the search paths for this linked lib.
#

# Set install dir
# ---------------
install(
	TARGETS ${XNCORE_LIB_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
