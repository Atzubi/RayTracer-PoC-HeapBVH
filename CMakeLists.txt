cmake_minimum_required(VERSION 3.12)
project(Heap_BVH)

SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pg -Ofast -march=native")

find_package (Threads)
find_package(OpenCL REQUIRED)

add_executable(Heap_BVH main.cpp OpenCL_Kernel.h)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})
find_package(SFML 2 REQUIRED network audio graphics window system)
include_directories(${SFML_INCLUDE_DIR})
target_link_libraries(Heap_BVH ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
target_link_libraries (Heap_BVH ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(Heap_BVH OpenCL::OpenCL)