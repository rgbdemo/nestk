# ----------------------------------------------------------------------------
#  CMake file for libpng. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------
project(vectorial)

# List of C++ files:

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB lib_hdrs include/vectorial/*.h)

# ----------------------------------------------------------------------------------
# 				Define the library target:
# ----------------------------------------------------------------------------------

install(FILES ${lib_hdrs} DESTINATION include/vectorial)
