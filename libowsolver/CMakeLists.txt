cmake_minimum_required(VERSION 3.20)
project(owsolver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(GLOB_RECURSE SOLVER_SOURCES src/*.cpp)

add_library(owsolver STATIC ${SOLVER_SOURCES})

add_library(OWSolver::owsolver ALIAS owsolver)

target_include_directories(owsolver PUBLIC include/ ${BULLET_INCLUDE_DIRS})

target_link_libraries(owsolver PUBLIC ${BULLET_LIBRARIES})

if(TARGET glm::glm)
    target_link_libraries(owsolver PUBLIC glm::glm)
else()
    target_include_directories(owsolver PUBLIC ${GLM_INCLUDE_DIRS})
endif()