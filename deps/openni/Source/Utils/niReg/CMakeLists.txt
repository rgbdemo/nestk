#project(niReg)
set(REG_EXE niReg)

file(GLOB ${REG_EXE}_sources ./*.cpp)

include_directories(${OPENNI_INCLUDE_DIR})

add_executable(${REG_EXE} ${${REG_EXE}_sources})

target_link_libraries(${REG_EXE} ${OPENNI_LIB_NAME})
add_dependencies(${REG_EXE} ${OPENNI_LIB_NAME})
