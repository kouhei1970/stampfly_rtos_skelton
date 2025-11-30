cmake_minimum_required(VERSION 3.14)
project(eskf_debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ESP-IDF types stub
add_library(esp_stubs INTERFACE)
target_include_directories(esp_stubs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

# Math library (header-only)
add_library(stampfly_math INTERFACE)
target_include_directories(stampfly_math INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/stampfly_math/include
)

# ESKF library (copied from original with ESP-IDF dependencies removed)
add_library(stampfly_eskf
    eskf_pc.cpp
)
target_include_directories(stampfly_eskf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/stampfly_eskf/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(stampfly_eskf PUBLIC stampfly_math esp_stubs)

# Main debug executable (simple test)
add_executable(eskf_debug main.cpp)
target_link_libraries(eskf_debug PRIVATE stampfly_eskf)

# Replay tool (run ESKF on recorded sensor data)
add_executable(eskf_replay replay.cpp)
target_link_libraries(eskf_replay PRIVATE stampfly_eskf)
