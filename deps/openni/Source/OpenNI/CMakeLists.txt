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
        set(OPENNI_LINK_LIBRARIES "-framework CoreFoundation -framework IOKit")
elseif(OPENNI_PLATFORM_WIN32)
        #SET(OPENNI_DRIVER_DIR ${OpenNI_SOURCE_DIR}/Platform/Win32/Driver/Build/sys)
        file(GLOB ${OPENNI_LIB_NAME}_platform_sources ./Win32/*.cpp ${OPENNI_DRIVER_DIR}/*.c)
        MESSAGE("SOURCE DIR" ${${OPENNI_LIB_NAME}_platform_sources})
        #INCLUDE_DIRECTORIES(${OPENNI_DRIVER_DIR})
        #INCLUDE_DIRECTORIES("C:/WinDDK/7600.16385.1/inc/ddk" "C:/WinDDK/7600.16385.1/inc/api")
        SET(OPENNI_LINK_LIBRARIES "wsock32")
endif()

# Set our target
# ---------------

add_library(
	${OPENNI_LIB_NAME} 
        SHARED ${${OPENNI_LIB_NAME}_sources}
${${OPENNI_LIB_NAME}_platform_sources}
)

# Set Link libraries.
# -------------------
IF(UNIX)
  find_library(LIBDL dl)
ENDIF()
target_link_libraries(
	${OPENNI_LIB_NAME}
	${USB_LIBRARY} 
	${LIBDL} 
        ${TINYXML_LIB_NAME}
        ${OPENNI_LINK_LIBRARIES}
)

# Set path where to install the target, lib, runtime. 
# (to same dir where we call cmake ., as we use the "dot")
# -----------------------------------------------------
install(TARGETS ${OPENNI_LIB_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
