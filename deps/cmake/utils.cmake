#
# INSTALL_NOBASE_HEADER_FILES(prefix file file file ...)
# Will create install rules for those files of the list
# which are headers (.h or .txx).
# If .in files are given, the .in extension is removed.
#

MACRO(NESTK_INSTALL_NOBASE_HEADER_FILES prefix)
FOREACH(file ${ARGN})
  IF(${file} MATCHES "\\.(h|txx|hxx|hpp|ipp)(\\.in)?$")
    STRING(REGEX REPLACE "\\.in$" "" install_file ${file})
    GET_FILENAME_COMPONENT(dir ${install_file} PATH)
    STRING(REGEX REPLACE "^${CMAKE_INSTALL_PREFIX}/$" "" install_file ${file})
    #INSTALL_FILES(${prefix}/${dir} FILES ${install_file}) ### deprecated
    INSTALL(FILES ${install_file} DESTINATION ${prefix}/${dir})
  ENDIF(${file} MATCHES "\\.(h|txx|hxx|hpp|ipp)(\\.in)?$")
ENDFOREACH(file ${filelist})
ENDMACRO(NESTK_INSTALL_NOBASE_HEADER_FILES)

MACRO(QT4_WRAP_CPP_INLINE targetname )
# get include dirs
GET_DIRECTORY_PROPERTY(moc_includes_tmp INCLUDE_DIRECTORIES)
SET(moc_includes)
FOREACH(it ${moc_includes_tmp})
  SET(moc_includes ${moc_includes} "-I${it}")
ENDFOREACH(it)

FOREACH(it ${ARGN})
  GET_FILENAME_COMPONENT(it ${it} ABSOLUTE)
  GET_FILENAME_COMPONENT(outfile ${it} NAME_WE)

  #SET(infile ${CMAKE_CURRENT_SOURCE_DIR}/${it})
  SET(outfile ${CMAKE_CURRENT_BINARY_DIR}/${outfile}.moc)
  ADD_CUSTOM_TARGET(${targetname}
                    ${QT_MOC_EXECUTABLE} ${moc_includes} -i -o ${outfile} ${it}
                   )
ENDFOREACH(it)
ENDMACRO(QT4_WRAP_CPP_INLINE)

SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)

MACRO(INSTALL_STANDALONE_BUNDLE target libdirs)
IF (APPLE)
INSTALL(TARGETS ${target}
    BUNDLE DESTINATION . COMPONENT Runtime
    RUNTIME DESTINATION bin COMPONENT Runtime
)
SET(plugin_dest_dir bin)
SET(qtconf_dest_dir bin)
IF(APPLE)
  SET(plugin_dest_dir ${target}.app/Contents/MacOS)
  SET(qtconf_dest_dir ${target}.app/Contents/Resources)
  SET(APPS "\${CMAKE_INSTALL_PREFIX}/${target}.app")
ENDIF(APPLE)
IF(WIN32)
  SET(APPS "\${CMAKE_INSTALL_PREFIX}/bin/${target}.exe")
ENDIF(WIN32)

INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT Runtime)

INSTALL(CODE "
    file(WRITE \"\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"\")
    " COMPONENT Runtime)

INSTALL(CODE "
   file(GLOB_RECURSE QTPLUGINS
      \"\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
   include(BundleUtilities)
   fixup_bundle(\"${APPS}\"   \"\"   \"${libdirs}\")
   " COMPONENT Runtime)
ENDIF(APPLE)
ENDMACRO(INSTALL_STANDALONE_BUNDLE)

MACRO(NESTK_ADD_EXECUTABLE target)
IF (BUILD_MACOSX_BUNDLE)
   ADD_EXECUTABLE(${target} MACOSX_BUNDLE ${ARGN})
   SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
   INSTALL_STANDALONE_BUNDLE(${target} ${DIRS})
ELSE()
   ADD_EXECUTABLE(${target} ${ARGN})
ENDIF()
INSTALL(TARGETS ${target} DESTINATION bin)
ENDMACRO(NESTK_ADD_EXECUTABLE)

IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* 
    OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64*
    OR CMAKE_SIZEOF_VOID_P MATCHES "8")
  SET(ARCHITECTURE_IS_X86_64 1 CACHE INTERNAL "64 bit architecture")
ENDIF()
