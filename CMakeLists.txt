cmake_minimum_required(VERSION 3.5.0)
project(nfc_preliminary VERSION 0.1.0 LANGUAGES C CXX)



## CHANGE THE FOLLOWING AS YOU WANT
####################################################################################################
option(BUILD_EXECUTABLES "Enable the compilation of the adjoint test files." ON) # ON/  OFF
option(BUILD_TESTS "Enable the compilation of the test files." OFF)                # ON/  OFF
#### Library subdir
add_subdirectory(FD2D)


#### nfc "lib"
add_subdirectory(nfc_2025)

#### Add executables 
if (BUILD_EXECUTABLES)
  add_subdirectory(${CMAKE_SOURCE_DIR}/executables)
  message(STATUS "BUILD EXECUTABLES = ON")
endif()


#### Add executables from the "tests" folder
if (BUILD_TESTS)
  add_subdirectory(${CMAKE_SOURCE_DIR}/tests)
  message(STATUS "BUILD TEST = ON")
endif()






