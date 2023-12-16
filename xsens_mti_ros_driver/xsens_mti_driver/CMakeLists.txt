project( xsens_mti_driver )

cmake_minimum_required(VERSION 3.9)

message("Project ${PROJECT_NAME}")

aux_source_directory( src XSENS_MTI_DRIVER_SRC )
add_library(${PROJECT_NAME} STATIC ${XSENS_MTI_DRIVER_SRC})

list( APPEND CXX_CUSTOM_FLAGS -std=c++17 -Wall -Wpedantic -Wno-narrowing -Wno-deprecated -O3 -ffast-math -fPIC )

if(NOT CMAKE_BUILD_TYPE)
    message("Build type automatically set to 'Release'")
    set(CMAKE_BUILD_TYPE "Release" )
else(NOT CMAKE_BUILD_TYPE)
    message("Build type: '${CMAKE_BUILD_TYPE}'")
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        message(WARNING "WARNING: you will get poor performance!")
    endif(CMAKE_BUILD_TYPE STREQUAL "Debug")
endif(NOT CMAKE_BUILD_TYPE)

# Finally, set the compilers optpions
target_compile_options(${PROJECT_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CXX>:${CXX_CUSTOM_FLAGS}>)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_library( RT_LIB rt )

target_link_libraries( ${PROJECT_NAME} PUBLIC ${RT_LIB} )

target_include_directories( ${PROJECT_NAME} PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

if( BUILD_EXAMPLES )
    add_subdirectory(examples)
endif( BUILD_EXAMPLES )
