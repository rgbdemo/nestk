cmake_minimum_required(VERSION 2.8)

project(SensorKinect)

# Check the system
# ----------------
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(SENSOR_PLATFORM_LINUX true)
elseif(WIN32)
	set(SENSOR_PLATFORM_WIN32 true)
elseif(APPLE)
	set(SENSOR_PLATFORM_MAC true)
endif()

set(XNCORE_LIB_NAME "XnCore")
set(XNDDK_LIB_NAME "XnDDK")
set(XNFORMAT_LIB_NAME "XnFormats")
set(XNDEVICEFILE_LIB_NAME "XnDeviceFile")
set(XNDEVICESENSOR_LIB_NAME "XnDevicesSensorV2")
set(XNSENSORSERVER_TARGET_NAME "XnSensorServer")
set(OPENNI_INCLUDE_DIR "${OPENNI_BASE_DIR}/Include/")
set(OPENNI_LIB_DIR "${OPENNI_BASE_DIR}/Platform/Mac/Bin/Release/")
set(LIBJPEG_INCLUDE_DIR "${OPENNI_BASE_DIR}/Source/External/LibJPEG/")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/Include)

# Add the sub-cmakefiles.txt
# --------------------------
add_subdirectory("./Source/XnCore/")
add_subdirectory("./Source/XnFormats/")
add_subdirectory("./Source/XnDDK/")
add_subdirectory("./Source/XnDeviceFile/")
add_subdirectory("./Source/XnDeviceSensorV2/")
add_subdirectory("./Source/Utils/XnSensorServer/")
