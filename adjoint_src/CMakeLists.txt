cmake_minimum_required(VERSION 3.5.0)
project(nfc_preliminary_adjoint VERSION 0.1.0 LANGUAGES C CXX)

file(GLOB_RECURSE source_files "src/*.cpp")
file(GLOB_RECURSE header_files "include/*.h")
add_library(nfc_preliminary_adjoint STATIC ${source_files} ${header_files})
target_include_directories(nfc_preliminary_adjoint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nfc_preliminary_adjoint nfc_preliminary)
target_link_libraries(nfc_preliminary_adjoint cppad_lib)