cmake_minimum_required(VERSION 3.10)
project(HybridAstar)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# path to OMPL installation directory
set(OMPL_DIR  /usr/share/ompl/cmake)  # Adjust this path as per your installation

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# # Find OMPL
find_package(OMPL REQUIRED PATHS ${OMPL_DIR})
include_directories(${OMPL_INCLUDE_DIRS})

# Add subdirectories
add_subdirectory(common)
add_subdirectory(core)

# Add executable
add_executable(main main.cpp)

# Link libraries to the main executable
target_link_libraries(main MapLoader HybridAstar pathProcessing ${OpenCV_LIBS}  ${OMPL_LIBRARIES})
