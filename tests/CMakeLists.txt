add_executable(core_test core_test.cpp)
target_link_libraries(core_test PRIVATE localization_amcl_core)
add_test(NAME core_test COMMAND core_test WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
