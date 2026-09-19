cmake_minimum_required(VERSION 3.16)
project(ACE_Tracking LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ace_tracking
    src/tracking/main.c
    src/tracking/ball_detector.cpp
)

target_include_directories(ace_tracking PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(ace_tracking PRIVATE ${OpenCV_LIBS} Threads::Threads)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(ace_motor_test
        src/drone_control/motor_test.c
        src/drone_control/motion.c
        src/drone_control/stepper.c
    )
    target_include_directories(ace_motor_test PRIVATE src/drone_control)
    target_link_libraries(ace_motor_test PRIVATE Threads::Threads m)

    add_executable(ace_drone
        src/drone_control/drone_main.c
        src/drone_control/navigator.c
        src/drone_control/calibrate.c
        src/drone_control/kinematics.c
        src/drone_control/motion.c
        src/drone_control/stepper.c
        src/tracking/ball_detector.cpp
    )
    target_include_directories(ace_drone PRIVATE
        src/drone_control
        src/tracking
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(ace_drone PRIVATE ${OpenCV_LIBS} Threads::Threads m)
else()
    message(STATUS "ace_motor_test und ace_drone werden uebersprungen (nur Linux/Raspberry Pi).")
endif()
