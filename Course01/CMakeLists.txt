project(course01)

add_executable(course01 main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(course01 ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})