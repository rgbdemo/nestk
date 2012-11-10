#project(Sample-NiAudioSample)
set(AUDIOSAMPLE_EXE Sample-NiAudioSample)

file(GLOB ${AUDIOSAMPLE_EXE}_sources ./*.cpp)

include_directories(${OPENNI_INCLUDE_DIR})

add_executable(${AUDIOSAMPLE_EXE} ${${AUDIOSAMPLE_EXE}_sources})

target_link_libraries(${AUDIOSAMPLE_EXE} ${OPENNI_LIB_NAME})
target_link_libraries(${AUDIOSAMPLE_EXE} ${TINYXML_LIB_NAME})

link_directories(${CMAKE_CURRENT_SOURCE_DIR})
#set_target_properties(
#	${AUDIOSAMPLE_EXE} PROPERTIES 
#	INSTALL_RPATH ${LIB_INSTALL_PATH}
 #	INSTALL_RPATH_USE_LINK_PATH true	
#	)
#set_target_properties(${AUDIOSAMPLE_EXE} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
#set(CMAKE_SKIP_RPATH false)
#set(CMAKE_INSTALL_RPATH "./lib/")
#set_target_properties(${AUDIOSAMPLE_EXE} PROPERTIES LINK_FLAGS " -rpath ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_PATH}/libOpenNI.dylib")
#SET(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib") 
#set_target_properties(${AUDIOSAMPLE_EXE} PROPERTIES 
#        BUILD_WITH_INSTALL_RPATH ON
#	        INSTALL_NAME_DIR "@executable_path/lib"
#		)
