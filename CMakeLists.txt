cmake_minimum_required(VERSION 3.10)
project(MotionPlanner)

# Recursively find all .cpp files under src/
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

# Include directories for Eigen, nanoflann, and your source headers
include_directories(
    src
    third_party/eigen
    third_party/nanoflann
)

# Create the main executable
add_executable(motion_planner ${SRC_FILES})

# Set C++ standard and compile flags
set_target_properties(motion_planner PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Optional: add extra compiler flags if needed
if(MSVC)
    target_compile_options(motion_planner PRIVATE /W4)
else()
    target_compile_options(motion_planner PRIVATE -Wall -O2 -pthread -Wno-reorder)
endif()

option(BUILD_TEST "Build a test binary from a single test source file" OFF)
set(TEST_FILE "" CACHE FILEPATH "Path to test file to build (e.g. test/testAll.cpp)")

if(BUILD_TEST AND TEST_FILE)
    get_filename_component(test_name "${TEST_FILE}" NAME_WE)
    add_executable(${test_name} ${TEST_FILE} ${SRC_FILES})
    target_include_directories(${test_name} PRIVATE src third_party/eigen third_party/nanoflann)
    set_target_properties(${test_name} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    if(MSVC)
        target_compile_options(${test_name} PRIVATE /W4)
    else()
        target_compile_options(${test_name} PRIVATE -Wall -O2 -pthread -Wno-reorder)
    endif()
    message(STATUS "Added test executable: ${test_name} from ${TEST_FILE}")
endif()