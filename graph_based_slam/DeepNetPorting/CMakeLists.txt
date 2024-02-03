cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(DeepProject)


set(CMAKE_BUILD_TYPE "RELEASE")

LIST(APPEND CMAKE_PREFIX_PATH "/home/navid/eufs_ws/lib/libtorch/share/cmake/Torch")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})


#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(TensorRT_DIR "/usr/src/tensorrt")

find_package(Torch REQUIRED)

message(STATUS "Find Torch VERSION: ${Torch_VERSION}")
add_definitions(-DTORCH_VERSION_MAJOR=${Torch_VERSION_MAJOR})
add_definitions(-DTORCH_VERSION_MINOR=${Torch_VERSION_MINOR})
add_definitions(-DTORCH_VERSION_PATCH=${Torch_VERSION_PATCH})

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)


message("Dirs = ${TensorRT_LIBRARIES}")

include_directories(
  include
  ${TORCH_INCLUDE_DIRECTORIES}
  ${OpenCV_INCLUDE_DIRS}
)


find_package(TensorRT REQUIRED)
#find_package(torchtrt REQUIRED)
set(TorchTensorRT_INCLUDE_DIR "/home/navid/eufs_ws/lib/torch_tensorrt/include")
set(TorchTensorRT_LIBS "/home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt.so;/home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt_plugins.so;/home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt_runtime.so")

#------
aux_source_directory( src DEEPNETPORTING_SRC )
add_library(yolo_lib SHARED src/detector.cpp src/utils.cpp)
target_include_directories(yolo_lib PUBLIC ${TorchTensorRT_INCLUDE_DIR})
target_link_libraries(yolo_lib ${TORCH_LIBRARIES} ${OpenCV_LIBS} )

install(TARGETS
  yolo_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS
#   yolo_lib
#   EXPORT
#   yolo_lib
#   LIBRARY DESTINATION lib 
#   ARCHIVE DESTINATION lib 
#   RUNTIME DESTINATION bin 
#   INCLUDES DESTINATION
#   include
# )
#------

#############################################################

add_executable(torch_app 
src/resnet18_test.cpp
src/utils.cpp)
target_link_libraries(torch_app ${TORCH_LIBRARIES} ${OpenCV_LIBS})
set_property(TARGET torch_app PROPERTY CXX_STANDARD 14)


# find_package(TensorRT REQUIRED)
# #find_package(torchtrt REQUIRED)
# set(TorchTensorRT_INCLUDE_DIR "~/torch_tensorrt/include")
# set(TorchTensorRT_LIBS "~/torch_tensorrt/lib/libtorchtrt.so;~/torch_tensorrt/lib/libtorchtrt_plugins.so;~/torch_tensorrt/lib/lib/libtorchtrt_runtime.so")


#############################################################
add_executable(torchRT_app 
src/resnet18rt_test.cpp
src/utils.cpp)

target_include_directories(torchRT_app PUBLIC 
${TorchTensorRT_INCLUDE_DIR})

target_link_libraries(torchRT_app ${TORCH_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(torchRT_app -Wl,--no-as-needed /home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt_runtime.so )
target_link_libraries(torchRT_app
/home/navid/eufs_ws/lib/libtorch/lib/libtorch.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_cpu.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_cuda.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_global_deps.so
/home/navid/eufs_ws/lib/libtorch/lib/libbackend_with_compiler.so
/home/navid/eufs_ws/lib/libtorch/lib/libc10.so
/home/navid/eufs_ws/lib/libtorch/lib/libc10_cuda.so)
set_property(TARGET torchRT_app PROPERTY CXX_STANDARD 14)

#############################################################

add_executable(compiletorchRT_app 
src/optimization_resnet.cpp
src/utils.cpp)

target_include_directories(compiletorchRT_app PUBLIC 
${TorchTensorRT_INCLUDE_DIR})

target_link_libraries(compiletorchRT_app -Wl,--no-as-needed ${TORCH_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(compiletorchRT_app -Wl,--no-as-needed 
/home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt.so
/home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt_plugins.so)

target_link_libraries(compiletorchRT_app -Wl,--no-as-needed
/home/navid/eufs_ws/lib/libtorch/lib/libtorch.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_cpu.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_cuda.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_global_deps.so
/home/navid/eufs_ws/lib/libtorch/lib/libbackend_with_compiler.so
/home/navid/eufs_ws/lib/libtorch/lib/libc10.so
/home/navid/eufs_ws/lib/libtorch/lib/libc10_cuda.so
)
set_property(TARGET torchRT_app PROPERTY CXX_STANDARD 14)

#############################################################
add_executable(yolo_app 
src/main_detect.cpp
src/detector.cpp
src/utils.cpp)

target_include_directories(yolo_app PUBLIC 
${TorchTensorRT_INCLUDE_DIR})

target_link_libraries(yolo_app ${TORCH_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(yolo_app -Wl,--no-as-needed /home/navid/eufs_ws/lib/torch_tensorrt/lib/libtorchtrt_runtime.so )
target_link_libraries(yolo_app
/home/navid/eufs_ws/lib/libtorch/lib/libtorch.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_cpu.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_cuda.so
/home/navid/eufs_ws/lib/libtorch/lib/libtorch_global_deps.so
/home/navid/eufs_ws/lib/libtorch/lib/libbackend_with_compiler.so
/home/navid/eufs_ws/lib/libtorch/lib/libc10.so
/home/navid/eufs_ws/lib/libtorch/lib/libc10_cuda.so)
set_property(TARGET yolo_app PROPERTY CXX_STANDARD 14)



