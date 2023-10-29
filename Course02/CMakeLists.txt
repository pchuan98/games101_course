project(course02)

add_executable(course02 main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(course02 ${OpenCV_LIBRARIES} ${EIGEN3_LIBRARIES})