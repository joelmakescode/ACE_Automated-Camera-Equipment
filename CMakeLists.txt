cmake_minimum_required(VERSION 3.16)
project(ACE_Tracking LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Ohne Angabe hoechstens zwei Uebersetzungen gleichzeitig.
#
# ball_detector.cpp zieht <opencv2/opencv.hpp> herein, und eine solche
# Uebersetzung braucht rund ein Gigabyte. Auf einem Pi reicht das, um mit
# "make -j4" den Speicher zu sprengen; der OOM-Killer raeumt dann auf und
# nimmt gern die SSH-Sitzung mit. Wer mehr Kerne hat, kann es ueberstimmen:
#     cmake --build build --parallel 4
if(NOT DEFINED CMAKE_BUILD_PARALLEL_LEVEL AND NOT DEFINED ENV{CMAKE_BUILD_PARALLEL_LEVEL})
    set(ENV{CMAKE_BUILD_PARALLEL_LEVEL} 2)
endif()

# ---------------------------------------------------------------------------
# Gemeinsame Bausteine: jede Quelldatei genau einmal uebersetzen.
#
# Vorher stand ball_detector.cpp in drei Zielen und wurde entsprechend
# dreimal uebersetzt - bei paralleler Ausfuehrung also drei OpenCV-Laeufe
# nebeneinander. Als Bibliothek faellt genau eine davon an.
# ---------------------------------------------------------------------------

add_library(ace_vision STATIC
    src/tracking/ball_detector.cpp
)
target_include_directories(ace_vision PUBLIC
    src/tracking
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ace_vision PUBLIC ${OpenCV_LIBS} Threads::Threads)

# Die Ziele mit Kamera haben selbst nur C-Quellen, ziehen ueber ace_vision
# aber C++ herein. Ohne diese Angabe waehlt CMake den C-Linker, und die
# C++-Laufzeit fehlt beim Binden.
add_executable(ace_tracking src/tracking/main.c)
target_link_libraries(ace_tracking PRIVATE ace_vision)
set_target_properties(ace_tracking PROPERTIES LINKER_LANGUAGE CXX)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Antrieb, Kinematik und Bahnplanung. Kein OpenCV, nur Linux-GPIO.
    add_library(ace_core STATIC
        src/drone_control/stepper.c
        src/drone_control/motion.c
        src/drone_control/kinematics.c
        src/drone_control/path.c
        src/drone_control/figure.c
    )
    target_include_directories(ace_core PUBLIC src/drone_control)
    target_link_libraries(ace_core PUBLIC Threads::Threads m)

    add_executable(ace_motor_test src/drone_control/motor_test.c)
    target_link_libraries(ace_motor_test PRIVATE ace_core)

    # Figurenfahrt ohne Kamera: braucht kein OpenCV.
    add_executable(ace_figure src/drone_control/figure_main.c)
    target_link_libraries(ace_figure PRIVATE ace_core)

    # Objektverfolgung: Figurenfahrt-Kern plus Kamera und Bildmodell.
    add_executable(ace_track
        src/drone_control/track_main.c
        src/drone_control/visual.c
    )
    target_link_libraries(ace_track PRIVATE ace_core ace_vision)
    set_target_properties(ace_track PROPERTIES LINKER_LANGUAGE CXX)

    add_executable(ace_drone
        src/drone_control/drone_main.c
        src/drone_control/navigator.c
        src/drone_control/calibrate.c
    )
    target_link_libraries(ace_drone PRIVATE ace_core ace_vision)
    set_target_properties(ace_drone PROPERTIES LINKER_LANGUAGE CXX)
else()
    message(STATUS "ace_motor_test, ace_figure, ace_track und ace_drone "
                   "werden uebersprungen (nur Linux/Raspberry Pi).")
endif()
