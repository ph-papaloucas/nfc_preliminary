# Set the include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Define the library based on the build type
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CPPAD_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/Debug/cppad_lib.lib")
    message(STATUS "CMakeLists.txt(cppad): cppad_lib is at Debug because $CMAKE_BUILD_TYPE = Debug. Make sure that this is what you wanted")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CPPAD_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/Release/cppad_lib.lib")
else()
    message(STATUS "ERROR: At top lvl cmake declare variable ${CMAKE_BUILD_TYPE}=Release or Debug")
endif()

# Create an interface library for cppad
add_library(cppad_lib INTERFACE)

# Specify the library file for the interface library
target_link_libraries(cppad_lib INTERFACE "${CPPAD_LIB_PATH}")

# Include directory for the cppad library
target_include_directories(cppad_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

