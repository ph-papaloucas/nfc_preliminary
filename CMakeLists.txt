cmake_minimum_required(VERSION 3.5.0)
project(nfc_preliminary VERSION 0.1.0 LANGUAGES C CXX)



## CHANGE THE FOLLOWING AS YOU WANT
####################################################################################################
option(CLEAR_CACHE "Clear the CMake cache before configuring." OFF) # ON EVERYTIME YOU CHANGE SOMETHING OF THE ABOVE

option(BUILD_MAIN_EXECUTABLES "Enable the compilation of the main executables files." ON)                # ON/  OFF
option(BUILD_TESTS "Enable the compilation of the test files." ON)                # ON/  OFF
option(BUILD_ADJOINT_TESTS "Enable the compilation of the adjoint test files." ON) # ON/  OFF
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
#####################################################################################################


if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
    add_definitions(-DVERBOSITY_LEVEL=2)  # Set verbosity level for Debug builds
endif()


#### Handle Externals
add_subdirectory(externals/cppad)

#### Add source code
add_subdirectory(Source)

### Add executables
if (BUILD_MAIN_EXECUTABLES)
add_subdirectory(${CMAKE_SOURCE_DIR}/main_executables)
  message(STATUS "BUILD MAIN EXECUTABLES = ON")
endif()

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




