cmake_minimum_required(VERSION 2.6)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(flann)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

include(${PROJECT_SOURCE_DIR}/cmake/flann_utils.cmake)
set(FLANN_VERSION 1.6.8)
DISSECT_VERSION()
GET_OS_INFO()

# Set the build type.  Options are:
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

find_hdf5()
if (USE_MPI OR HDF5_IS_PARALLEL)
    find_package(MPI)
endif()

if (HDF5_IS_PARALLEL)
    if (NOT MPI_FOUND)
        message(ERROR "Found the parallel HDF5 library, but could not find the MPI library. Define the MPI_COMPILER variable to the path of your MPI compiler.")
    endif()
    # Parallel HDF5 needs to find the "mpi.h" header file
    include_directories(${MPI_INCLUDE_PATH})
endif()

if (USE_MPI)
    if (NOT MPI_FOUND)
        message(ERROR "Could not find an MPI library. Define the MPI_COMPILER variable to the path of your MPI compiler.")
    endif()

    if (NOT HDF5_IS_PARALLEL)
        message(ERROR "For MPI support the Parallel HDF5 library is required.")
    endif()

endif(USE_MPI)

#set the C/C++ include path to the "include" directory
include_directories(${PROJECT_SOURCE_DIR}/src/cpp)

# require proper c++
#add_definitions( "-Wall -ansi -pedantic" )
# HDF5 uses long long which is not ansi

add_subdirectory( cmake )
add_subdirectory( src )

