# Copyright © 2022 Zen Shawn. All rights reserved. Created at 21:31:13, June 17,
# 2022

cmake_minimum_required(VERSION 3.13)

option(BUILD_SHARED_LIBS "build shared libs" OFF)
set(CMAKE_BUILD_TYPE
    "RelWithDebInfo"
    CACHE STRING "build type, Release/Debug/MinSizeRel/RelWithDebInfo")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

project(
  SceneGraph
  VERSION 0.0.0
  DESCRIPTION "Instroduction about SceneGraph")
set(CMAKE_INSTALL_PREFIX
    "$ENV{HOME}/Programs/SceneGraph"
    CACHE PATH "install prefix")
include(GNUInstallDirs)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Found ccache")
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

find_package(Eigen3 REQUIRED)

add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)
include_directories(Src)
add_subdirectory(Src)
add_subdirectory(Examples)
