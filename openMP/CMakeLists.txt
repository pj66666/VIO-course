cmake_minimum_required(VERSION 3.10)
project(OpenMPExample)

# Find OpenMP
find_package(OpenMP REQUIRED)

# Set C++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add executable
add_executable(main main.cpp)

# Add OpenMP flags
target_link_libraries(main PRIVATE OpenMP::OpenMP_CXX)
