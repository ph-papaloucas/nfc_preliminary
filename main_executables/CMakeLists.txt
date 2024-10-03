set(EXECUTABLES_OUTPUT_PATH "${CMAKE_BINARY_DIR}/1_executables")
file(GLOB executable_files "src/*.cpp")
  # Specify include directories

foreach(executable ${executable_files})
    get_filename_component(exe_name ${executable} NAME_WE)
    add_executable(${exe_name} ${executable})
    set_target_properties(${exe_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLES_OUTPUT_PATH})
    target_link_libraries(${exe_name} nfc_preliminary)
endforeach()