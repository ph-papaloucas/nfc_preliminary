set(EXECUTABLES_OUTPUT_PATH "${CMAKE_BINARY_DIR}/1-main-exe")
file(GLOB executable_files ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
  # Specify include directories

foreach(executable ${executable_files})
    get_filename_component(exe_name ${executable} NAME_WE)
    add_executable(${exe_name} ${executable})
    set_target_properties(${exe_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLES_OUTPUT_PATH})
    target_link_libraries(${exe_name} FD2D nfc_2025) #aero_adjoint
endforeach()