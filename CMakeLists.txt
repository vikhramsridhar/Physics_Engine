cmake_minimum_required(VERSION 3.10)
project(physics_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_executable(physics_sim
    src/main.cpp
    src/rigidbody.cpp
    src/collision.cpp
    src/solver.cpp
    src/world.cpp
)

target_include_directories(physics_sim PRIVATE include)

# ---------------------------------------------------------------------------
# Windowed viewer (physics_sim_gl) -- only built if GLFW + OpenGL are found,
# so the dependency-free console demo above always builds regardless.
# Install GLFW first:
#   Ubuntu/Debian:  sudo apt install libglfw3-dev
#   macOS (brew):    brew install glfw
#   Windows (vcpkg): vcpkg install glfw3
# ---------------------------------------------------------------------------
find_package(OpenGL)
find_package(glfw3 QUIET)

if(OpenGL_FOUND AND glfw3_FOUND)
    add_executable(physics_sim_gl
        src/render/main_gl.cpp
        src/render/renderer.cpp
        src/rigidbody.cpp
        src/collision.cpp
        src/solver.cpp
        src/world.cpp
    )
    target_include_directories(physics_sim_gl PRIVATE include)
    target_link_libraries(physics_sim_gl PRIVATE glfw OpenGL::GL)
    message(STATUS "GLFW + OpenGL found -- building physics_sim_gl viewer")
else()
    message(STATUS "GLFW not found -- skipping physics_sim_gl viewer "
                    "(install libglfw3-dev / glfw3 and re-run cmake to enable it)")
endif()
