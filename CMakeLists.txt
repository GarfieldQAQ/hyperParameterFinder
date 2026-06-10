cmake_minimum_required(VERSION 3.20)

project(hyperParameterFinder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(hyperParameterFinder
  src/main.cpp
)

if (MSVC)
  target_compile_options(hyperParameterFinder PRIVATE /W4 /permissive-)
else()
  target_compile_options(hyperParameterFinder PRIVATE -Wall -Wextra -Wpedantic)
endif()
