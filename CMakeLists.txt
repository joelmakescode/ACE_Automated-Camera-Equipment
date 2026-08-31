cmake_minimum_required(VERSION 3.16)
project(ACE_Tracking LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(ace_tracking
    src/tracking/main.c
    src/tracking/ball_detector.cpp
)

target_include_directories(ace_tracking PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ace_tracking PRIVATE ${OpenCV_LIBS})