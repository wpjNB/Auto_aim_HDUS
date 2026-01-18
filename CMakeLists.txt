cmake_minimum_required(VERSION 3.10)
project(HDUS VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/AngleSolver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/ArmorDetector/include
    ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/include
    ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Processor/include
    ${CMAKE_CURRENT_SOURCE_DIR}/general
    ${CMAKE_CURRENT_SOURCE_DIR}/Thread
)

# Collect source files
file(GLOB_RECURSE SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/AngleSolver/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ArmorDetector/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/general/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Thread/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Processor/src/*.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${SOURCES})

# Set HikVision camera library paths
set(HIKVISION_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/libs/linux)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        fmt::fmt 
        ${OpenCV_LIBS}
        ${Eigen3_LIBRARIES}
        ${HIKVISION_LIB_DIR}/libFormatConversion.so
        ${HIKVISION_LIB_DIR}/libMediaProcess.so
        ${HIKVISION_LIB_DIR}/libMvCameraControl.so
        ${HIKVISION_LIB_DIR}/libMVRender.so
        ${HIKVISION_LIB_DIR}/libMvUsb3vTL.so
        pthread
        m
)

# Set runtime path for shared libraries
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    INSTALL_RPATH "$ORIGIN:${HIKVISION_LIB_DIR}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# Optional: Add compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Print configuration summary
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")