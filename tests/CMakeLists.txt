set(TESTS_OUTPUT_PATH "${CMAKE_BINARY_DIR}/2_tests")
file(GLOB test_files "src/*.cpp")

foreach(test ${test_files})
    get_filename_component(test_name ${test} NAME_WE)
    add_executable(${test_name} ${test})
    set_target_properties(${test_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TESTS_OUTPUT_PATH})
    target_link_libraries(${test_name} aero nfc)
endforeach()
