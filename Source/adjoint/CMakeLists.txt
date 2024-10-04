
file(GLOB_RECURSE source_files "src/*.cpp")
file(GLOB_RECURSE header_files "include/*.h")
add_library(aero_adjoint STATIC ${source_files} ${header_files})
target_include_directories(aero_adjoint  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(aero_adjoint  aero)
target_link_libraries(aero_adjoint  cppad_lib)


