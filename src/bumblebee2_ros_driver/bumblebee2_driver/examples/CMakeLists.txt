message("Building examples")

find_package( Boost 1.30 COMPONENTS program_options REQUIRED )

add_executable(test_bb2 ${PROJECT_SOURCE_DIR}/examples/test_bb2.cpp)
target_include_directories( test_bb2 PRIVATE ${Boost_INCLUDE_DIR})
target_link_libraries(test_bb2 bumblebee2_driver ${Boost_LIBRARIES} )
set_target_properties(test_bb2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
