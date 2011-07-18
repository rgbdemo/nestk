PROJECT(Nite)

IF (APPLE)
  file(GLOB NITE_LIBS "Lib/MacOSX/*.dylib")
ELSEIF (UNIX)
  IF(ARCHITECTURE_IS_X86_64)
    file(GLOB NITE_LIBS "Lib/Linux64/*.so")
  ELSE()
    file(GLOB NITE_LIBS "Lib/Linux32/*.so")
  ENDIF()
ENDIF()

file(COPY ${NITE_LIBS} DESTINATION ${LIBRARY_OUTPUT_PATH})
# Fix Nite binaries RPATH on Mac.
IF (APPLE)
  execute_process(COMMAND sh "${CMAKE_CURRENT_SOURCE_DIR}/apple_fix_nite_libs.sh" WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
ENDIF()

ADD_SUBDIRECTORY(Samples/Players)
ADD_SUBDIRECTORY(Samples/CircleControl)
ADD_SUBDIRECTORY(Samples/Boxes)
install(FILES ${NITE_LIBS} DESTINATION lib)
