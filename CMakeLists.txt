cmake_minimum_required(VERSION 3.10)
project(Games101)

set(RootDir ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RootDir}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${RootDir}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${RootDir}/bin)

set(CMAKE_CXX_STANDARD 17)

if(MSVC)
    add_compile_options(/source-charset:utf-8 /execution-charset:utf-8)
endif()

set(OpenCV_DIR "D:\\opencv\\build")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(Eigen3_DIR "D:\\eigen\\bin\\share\\eigen3\\cmake")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

message(STATUS "[USER] RootDir = ${RootDir}")
message(STATUS "[USER] OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
message(STATUS "[USER] EIGEN3_LIBRARIES = ${EIGEN3_LIBRARIES}")
message(STATUS "[USER] IsMSVC = ${MSVC}")

add_subdirectory(course01)
add_subdirectory(course02)