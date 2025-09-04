# CMake build system for the Multi-Agent Transportation Project

# 1. Set minimum CMake version and project name
cmake_minimum_required(VERSION 3.10)
project(MultiAgentTransport CXX)

# 2. Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 3. Add include directories
include_directories(include)

# 4. Find required packages
# Note: You need to have OpenCV and CasADi installed on your system.
# For CasADi, you may need to set the 'casadi_DIR' variable to point to the
# directory containing 'casadi-config.cmake'.
# Example: cmake .. -Dcasadi_DIR=/path/to/casadi/lib/cmake/casadi

find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(casadi REQUIRED)
if (NOT casadi_FOUND)
    message(FATAL_ERROR "CasADi not found!")
endif()
include_directories(${casadi_INCLUDE_DIR})

# 5. Add executable target
add_executable(mpc_main
    src/main.cpp
    src/map.cpp
    src/planner.cpp
    src/nmpc.cpp
)

# 6. Link libraries
# The order of libraries might be important
target_link_libraries(mpc_main
    ${OpenCV_LIBS}
    ${casadi_LIBRARIES}
)

# 7. Installation (optional)
# install(TARGETS mpc_main DESTINATION bin)
