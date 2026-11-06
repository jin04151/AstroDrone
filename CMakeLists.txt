
cmake_minimum_required(VERSION 3.16)

project(drone_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

# ========================================
# UDP MAVLink transport (SITL)
# ========================================

add_library(mavlink_udp STATIC
    transport/udp/UdpMavlinkTransport.cpp
)

target_include_directories(mavlink_udp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/udp
)

target_compile_options(mavlink_udp PRIVATE
    -Wall
    -Wextra
)

# ========================================
# Serial MAVLink transport (REAL)
# ========================================

add_library(mavlink_serial STATIC
    transport/serial/SerialMavlinkTransport.cpp
)

target_include_directories(mavlink_serial PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/transport/serial
)

target_compile_options(mavlink_serial PRIVATE
    -Wall
    -Wextra
)

# ========================================
# Common control library
# ========================================

add_library(control_core STATIC
    src/MavlinkClient.cpp
    src/SafetyManager.cpp
    src/MissionManager.cpp
    src/HealthMonitor.cpp
    src/VisionReceiver.cpp
    src/FlightPhaseManager.cpp
)

target_include_directories(control_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(control_core SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_link_libraries(control_core PUBLIC
    mavlink_udp
)

target_compile_options(control_core PRIVATE
    -Wall
    -Wextra
)

# ========================================
# SITL application
# ========================================

add_executable(control_sitl
    apps/sitl/main.cpp
)

target_link_libraries(control_sitl PRIVATE
    control_core
    Threads::Threads
)

target_compile_options(control_sitl PRIVATE
    -Wall
    -Wextra
)

# ========================================
# REAL telemetry-only application
# ========================================

add_executable(control_real
    apps/real/main.cpp
)

target_include_directories(control_real SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_link_libraries(control_real PRIVATE
    mavlink_serial
    Threads::Threads
)

target_compile_options(control_real PRIVATE
    -Wall
    -Wextra
)
