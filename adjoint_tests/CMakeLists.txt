  set(TESTS_ADJOINT_OUTPUT_PATH "${CMAKE_BINARY_DIR}/3_adjoint_tests")
  file(GLOB adjoint_test_files "src/*.cpp")

  foreach(test ${adjoint_test_files})
    get_filename_component(test_name ${test} NAME_WE)
    add_executable(${test_name} ${test})

    # Set the runtime output directory for this specific executable
    set_target_properties(${test_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TESTS_ADJOINT_OUTPUT_PATH})

    target_link_libraries(${test_name} aero aero_adjoint)

  endforeach()
