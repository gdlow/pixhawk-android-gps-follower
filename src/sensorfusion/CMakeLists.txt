cmake_minimum_required(VERSION 3.0)
project(sensorfusion)

option(PRINT_DEBUG "Print debug messages" ON)

if(PRINT_DEBUG)
  add_definitions(-DDEBUG)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF) # Set OFF to use -std=c++11, otherwise it uses -std=gnu++11 by default

add_library(sensorfusion Commons.h GPSAccKalman.cpp GPSAccKalman.h Kalman.cpp Kalman.h Matrix.cpp Matrix.h)
