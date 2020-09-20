cmake_minimum_required(VERSION 3.1)

project(TMesh)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS " -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")

find_package(SFML REQUIRED COMPONENTS graphics window system )
find_package(glm REQUIRED)

set(TMESH_HEADERS include/frep2D.h
                  include/TMesh2D.h
                  include/TMeshRender2D.hpp
                  include/timer.hpp)

set(TMESH_SOURCES src/frep2D.cpp
                  src/TMesh2D.cpp)

add_executable(tmesh_sample  src/sample.cpp ${TMESH_HEADERS} ${TMESH_SOURCES})
target_link_libraries(tmesh_sample sfml-graphics sfml-window sfml-system)
