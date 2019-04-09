cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(custom_ops)

find_package(Torch REQUIRED)

add_executable(modelLoader modelLoader.cpp)
target_link_libraries(modelLoader "${TORCH_LIBRARIES}")
set_property(TARGET modelLoader PROPERTY CXX_STANDARD 11)
