file(GLOB source_files "src/*.cpp")
file(GLOB header_files "include/*.h")
add_library(aero STATIC ${source_files} ${header_files} ${externals_source_files} ${externals_header_files})
target_include_directories(aero PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(aero cppad_lib)


