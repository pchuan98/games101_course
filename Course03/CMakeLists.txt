cmake_minimum_required(VERSION 3.10)
project(course03)

add_executable(course03 main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(course03 ${OpenCV_LIBRARIES})