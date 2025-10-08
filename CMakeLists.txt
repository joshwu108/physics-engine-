cmake_minimum_required(VERSION 3.20)
project(ParallaxPhysics VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(APPLE)
        # ARM NEON optimizations for Apple Silicon
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native")
    endif()
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Source files 
set(SOURCES
    src/math/vec2.cpp
    src/math/mat2.cpp
    src/core/rigid_body.cpp
    src/collision/spatial_hash.cpp
    src/collision/collision_detector.cpp
)

# Create library 
add_library(parallax_physics STATIC ${SOURCES})

# Enable warnings
target_compile_options(parallax_physics PRIVATE
    -Wall -Wextra -Wpedantic
)
