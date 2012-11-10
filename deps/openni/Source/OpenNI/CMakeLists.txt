# Add search paths.
# ------------------
include_directories(${USB_INCLUDE_DIR})
include_directories(${TINYXML_INCLUDE_DIR})
include_directories(${OPENNI_INCLUDE_DIR})
include_directories("./FilesOnly")

# Get sources for OpenNI
# ----------------------
file(GLOB ${OPENNI_LIB_NAME}_sources ./*.cpp ./*.h ../../Include/*.h)

# Add platform specific sources.
# ------------------------------
if(OPENNI_PLATFORM_LINUX)
	file(GLOB ${OPENNI_LIB_NAME}_platform_sources ./Linux-x86/*.cpp)
elseif(OPENNI_PLATFORM_APPLE)
	file(GLOB ${OPENNI_LIB_NAME}_platform_sources ./Linux-x86/*.cpp)
	set(APPLE_FRAMEWORKS_LDFLAGS "-framework CoreFoundation -framework IOKit")
endif()

# Set our target
# ---------------
add_library(
	${OPENNI_LIB_NAME} 
	SHARED 
	${${OPENNI_LIB_NAME}_sources} 
	${${OPENNI_LIB_NAME}_platform_sources}
)

# Set Link libraries.
# -------------------
find_library(LIBDL dl)
target_link_libraries(
	${OPENNI_LIB_NAME}
	${USB_LIBRARY} 
	${LIBDL} 
	${TINYXML_LIB_NAME}
	${APPLE_FRAMEWORKS_LDFLAGS}
)

# Set path where to install the target, lib, runtime. 
# (to same dir where we call cmake ., as we use the "dot")
# -----------------------------------------------------
install(TARGETS ${OPENNI_LIB_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
