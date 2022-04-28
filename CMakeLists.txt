cmake_minimum_required (VERSION 2.8)

project(MonteCarloPathTracer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

add_compile_options(-pthread)

set(SOURCES
src/main.cpp
src/camera.cpp
src/vector.cpp
src/light.cpp
src/stringhandle.cpp
src/object.cpp
src/scene.cpp
src/tiny_obj_loader.cpp
src/myGrid.cpp
src/bvhnode.cpp
src/bvhtree.cpp)

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/extern)
set(EIGEN_DIR ${EXTERNAL_DIR}/eigen)
set(IGL_DIR ${EXTERNAL_DIR}/igl)

add_executable(MonteCarloPathTracer ${SOURCES})

target_include_directories(MonteCarloPathTracer PRIVATE ${EXTERNAL_DIR})
target_include_directories(MonteCarloPathTracer PRIVATE ${EIGEN_DIR})

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(MonteCarloPathTracer PRIVATE Threads::Threads)