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

# Find packages
find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SDL2_INCLUDE_DIRS}
)

# Source files
set(SOURCES
    src/main.cpp
    src/math/vec2.cpp
    src/math/mat2.cpp
    src/core/rigid_body.cpp
    src/core/world.cpp
    src/collision/spatial_hash.cpp
    src/collision/collision_detector.cpp
    src/constraints/constraint.cpp
    src/constraints/distance_constraint.cpp
    src/constraints/hinge_constraint.cpp
    src/constraints/spring_constraint.cpp
    src/constraints/motor_constraint.cpp
    src/solver/constraint_solver.cpp
    src/solver/island_solver.cpp
    src/renderer/renderer.cpp
    src/demos/demo_manager.cpp
)

# Create executable
add_executable(parallax_physics ${SOURCES})

# Link libraries
target_link_libraries(parallax_physics
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    pthread
)

# Enable warnings
target_compile_options(parallax_physics PRIVATE
    -Wall -Wextra -Wpedantic
)

# Optional: Add Metal support for GPU acceleration
if(APPLE)
    find_library(METAL_LIBRARY Metal)
    find_library(METALKIT_LIBRARY MetalKit)
    if(METAL_LIBRARY AND METALKIT_LIBRARY)
        target_link_libraries(parallax_physics ${METAL_LIBRARY} ${METALKIT_LIBRARY})
        target_compile_definitions(parallax_physics PRIVATE USE_METAL=1)
    endif()
endif()
