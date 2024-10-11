cmake_minimum_required(VERSION 3.5.0)
project(FD2D VERSION 0.1.0 LANGUAGES C CXX)



## CHANGE THE FOLLOWING AS YOU WANT
####################################################################################################
option(BUILD_TESTS "Enable the compilation of the test files." OFF)                # ON/  OFF
option(BUILD_ADJOINT_TESTS "Enable the compilation of the adjoint test files." OFF) # ON/  OFF
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
#####################################################################################################

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    add_definitions(-DVERBOSITY_LEVEL=3)  # Set verbosity level for Debug builds
endif()


#### Handle Externals
add_subdirectory(externals/cppad)

#### Add source code
add_subdirectory(Source)


#### create Flight Dynamics 2D library
add_library(FD2D INTERFACE)  # INTERFACE because it's a combined library
target_link_libraries(FD2D INTERFACE aero aero_adjoint)


#### Add executables from the "tests" folder
if (BUILD_TESTS)
  add_subdirectory(${CMAKE_SOURCE_DIR}/tests)
  message(STATUS "BUILD TEST = ON")
endif()


#### Add executables from the "adjoint_tests" folder
if (BUILD_ADJOINT_TESTS)
  add_subdirectory(${CMAKE_SOURCE_DIR}/adjoint_tests)
  message(STATUS "BUILD ADJOINT TESTS = ON")
endif()







