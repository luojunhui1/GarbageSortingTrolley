cmake_minimum_required(VERSION 2.8)

project(MASTER)
SET(CMAKE_BUILE_TYPE DEBUG)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g -std=c++14 -pthread -lv4l1 -lv4l2 -lrealsense2")
SET(BIN_NAME "run")

FIND_PACKAGE(OpenCV 4.4.0 REQUIRED)
FIND_PACKAGE(Threads)
FIND_PACKAGE(realsense2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(/usr/include/eigen3)
include_directories(${realsense2_INCLUDE_DIR})

MESSAGE(STATUS "Project:MASTER")
MESSAGE(STATUS "Project Directory: ${PROJECT_SOURCE_DIR}")
MESSAGE(STATUS "OpenCV Version: ${OpenCV_VERSION}")
MESSAGE(STATUS "OpenCV Libraries: ${OpenCVLIBS}")
MESSAGE(STATUS "Include Path:${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS}")

FILE(GLOB_RECURSE SOURCES "src/*.cpp")

#when "Drivers/DAHUA/src/*.cpp" is added to the sourcefiles at last line, these .cpp files looks like not been complied
ADD_EXECUTABLE(${BIN_NAME} main.cpp ${SOURCES})

#list(APPEND PSMOVEAPI_TRACKER_REQUIRED_LIBS v4l2)

TARGET_LINK_LIBRARIES(${BIN_NAME} ${realsense2_LIBRARY})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${CMAKE_THREAD_LIBS_INIT})
TARGET_LINK_LIBRARIES(${BIN_NAME} ${OpenCV_LIBS})
