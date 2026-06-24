cmake_minimum_required(VERSION 3.20)
project(obbyware)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE SOURCES src/*.cpp)

find_package(raylib REQUIRED)
find_package(Bullet REQUIRED)
find_package(glm REQUIRED)

add_executable(obbyware ${SOURCES} third_party/tinyxml2/tinyxml2.cpp)

add_subdirectory(libowsolver)

target_include_directories(obbyware PRIVATE
    include/
    third_party/tinyxml2/
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(obbyware PRIVATE
    raylib
    ${BULLET_LIBRARIES}
    glm::glm
    OWSolver::owsolver
)