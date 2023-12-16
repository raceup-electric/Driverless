message("Building examples")

add_executable(test_xsens_mti ${PROJECT_SOURCE_DIR}/examples/test_xsens_mti.cpp)
target_link_libraries(test_xsens_mti xsens_mti_driver)  
set_target_properties(test_xsens_mti PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
