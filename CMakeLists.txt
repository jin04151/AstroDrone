
cmake_minimum_required(VERSION 3.16)

project(control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(control
    src/main.cpp
    src/MavlinkClient.cpp
    src/SafetyManager.cpp
    src/MissionManager.cpp
    src/HealthMonitor.cpp
    src/VisionReceiver.cpp
    src/FlightPhaseManager.cpp
)

target_include_directories(control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(control SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_compile_options(control PRIVATE
    -Wall
    -Wextra
)
