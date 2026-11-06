
cmake_minimum_required(VERSION 3.16)

project(drone_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ========================================
# Common C++ source files
# ========================================

set(COMMON_SOURCES
    src/MavlinkClient.cpp
    src/SafetyManager.cpp
    src/MissionManager.cpp
    src/HealthMonitor.cpp
    src/VisionReceiver.cpp
    src/FlightPhaseManager.cpp
)

# ========================================
# Common control library
# ========================================

add_library(control_core STATIC
    ${COMMON_SOURCES}
)

target_include_directories(control_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(control_core SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_compile_options(control_core PRIVATE
    -Wall
    -Wextra
)

# ========================================
# SITL executable
# ========================================

add_executable(control_sitl
    apps/sitl/main.cpp
)

target_link_libraries(control_sitl PRIVATE
    control_core
)

target_compile_options(control_sitl PRIVATE
    -Wall
    -Wextra
)

# ========================================
# REAL executable
# ========================================
# Jetson + Pixhawk UART version
#
# Not implemented yet.
# Real flight TX must remain disabled.
#
# Future target:
# add_executable(control_real
#     apps/real/main.cpp
# )
