cmake_minimum_required(VERSION 3.5.0)
project(nfc_preliminary VERSION 0.1.0 LANGUAGES C CXX)

#prepare externals to add to library
file(GLOB_RECURSE externals_source_files "externals/maths/*.cpp")
file(GLOB_RECURSE externals_header_files "include/*.h")



# Add source code as a library
file(GLOB_RECURSE source_files "src/*.cpp")
file(GLOB_RECURSE header_files "include/*.h")
add_library(nfc_preliminary STATIC ${source_files} ${header_files} ${externals_source_files} ${externals_header_files})


# Add executables from the "single_executables" folder
file(GLOB executable_files "executable_src/*.cpp")
# Specify include directories
target_include_directories(nfc_preliminary PUBLIC ${CMAKE_SOURCE_DIR}/include)

foreach(executable ${executable_files})
  get_filename_component(exe_name ${executable} NAME_WE)
  add_executable(${exe_name} ${executable})
  target_link_libraries(${exe_name} nfc_preliminary)
endforeach()


# Add executables from the "tests" folder
file(GLOB test_files "tests/*.cpp")

foreach(test ${test_files})
  get_filename_component(test_name ${test} NAME_WE)
  add_executable(${test_name} ${test})
  target_link_libraries(${test_name} nfc_preliminary)
endforeach()



