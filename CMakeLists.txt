project(MonteCarloPathTracer)

set(CMAKE_CXX_STANDARD 11)

set(SOURCES
src/main.cpp
src/camera.h
src/camera.cpp
src/vector.h
src/vector.cpp
src/light.h
src/light.cpp
src/stringhandle.h
src/stringhandle.cpp
src/object.h
src/scene.h
src/stb_image_write.h
src/tiny_obj_loader.h
src/myGrid.h)

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(MonteCarloPathTracer ${SOURCES})

target_link_libraries(MonteCarloPathTracer libtinyobjloader.a)

