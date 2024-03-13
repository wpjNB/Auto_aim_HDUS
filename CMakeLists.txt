cmake_minimum_required(VERSION 3.10)
project(HDUS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)


find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
    ./AngleSolver/include
    ./ArmorDetector/include
    ./HKCamera/include
    ./SerialPort/include
)

file(GLOB_RECURSE source

    AngleSolver/src/*.*
    ArmorDetector/src/*.*
    HKCamera/src/*.*
    SerialPort/src/*.*
)
add_executable(HDUS ./main.cpp ${source})

target_link_libraries(
        HDUS
        ${OpenCV_LIBS}
        ${Eigen3_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/libs/linux/libFormatConversion.so
        ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/libs/linux/libMediaProcess.so
        ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/libs/linux/libMvCameraControl.so
        ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/libs/linux/libMVRender.so
        ${CMAKE_CURRENT_SOURCE_DIR}/HKCamera/libs/linux/libMvUsb3vTL.so
        -lpthread -lm
        )