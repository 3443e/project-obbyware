cmake_minimum_required(VERSION 3.20)
project(obbyware)
set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCES src/*.cpp)

find_package(raylib REQUIRED)
find_package(Bullet REQUIRED)

add_executable(obbyware ${SOURCES})

target_include_directories(obbyware PRIVATE
    include/
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(obbyware PRIVATE
    raylib
    ${BULLET_LIBRARIES}
)