project(MonteCarloPathTracer)

set(CMAKE_CXX_STANDARD 11)

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

add_executable(MonteCarloPathTracer ${SOURCES})
