cmake_minimum_required(VERSION 3.10)
project(test)

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PARENT_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PARENT_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PARENT_DIR}/bin)

if (MSVC)
    add_compile_options(/source-charset:utf-8 /execution-charset:utf-8)
endif()

set(OpenCV_DIR "D:\\opencv\\build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(Eigen3_DIR "D:\\eigen\\bin\\share\\eigen3\\cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(test main.cpp)

# add link library
target_link_libraries(test ${OpenCV_LIBS})

message(STATUS "[USER] PARENT_DIR = ${PARENT_DIR}")
message(STATUS "[USER] OpenCV_INCLUDE_DIRS = ${OpenCV_LIBS}")
message(STATUS "[USER] OpenCV_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "[USER] IsMSVC = ${MSVC}")