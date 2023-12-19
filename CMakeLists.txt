cmake_minimum_required(VERSION 3.10.0)
project(btpg)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(btpg ${SOURCES})