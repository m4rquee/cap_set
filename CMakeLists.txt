cmake_minimum_required(VERSION 3.21)

project(cap_set_ilp
        VERSION 1.0
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

# Set a default build type if none was specified
set(default_build_type "Release")

# Headers directories
include_directories("include")
include_directories(/opt/gurobi1100/linux64/include)

# Gurobi lib directory
link_directories(/opt/gurobi1100/linux64/lib)

# Compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wunused-local-typedefs")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    MESSAGE("Activating -Ofast optimizations")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -ffast-math")
endif ()

add_executable(cap_set_solver cap_set.cpp)
target_link_libraries(cap_set_solver m pthread)
target_link_libraries(cap_set_solver gurobi_c++ gurobi110)
