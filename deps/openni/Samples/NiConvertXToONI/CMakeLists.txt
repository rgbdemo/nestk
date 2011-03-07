#project(Sample-NiConvertXToONI)
set(CONVERTXTOONI_EXE Sample-NiConvertXToONI)

file(GLOB ${CONVERTXTOONI_EXE}_sources ./*.cpp)
include_directories(${OPENNI_INCLUDE_DIR})

add_executable(${CONVERTXTOONI_EXE} ${${CONVERTXTOONI_EXE}_sources})

target_link_libraries(${CONVERTXTOONI_EXE} ${OPENNI_LIB_NAME})
#target_link_libraries(${CONVERTXTOONI_EXE} ${TINYXML_LIB_NAME})
add_dependencies(${CONVERTXTOONI_EXE} ${OPENNI_LIB_NAME})
