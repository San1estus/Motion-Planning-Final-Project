cmake_minimum_required(VERSION 3.10...3.30)
project(RRT_car VERSION 1.0.0)

cmake_policy(SET CMP0074 NEW)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(RRT_car
                src/config.h 
                src/main.cpp 
                src/glad.c)

target_include_directories(RRT_car PRIVATE dependencies includes src)
target_compile_definitions(${PROJECT_NAME} PRIVATE SHADER_DIR="${CMAKE_SOURCE_DIR}/shaders/")
target_link_libraries(RRT_car glfw OpenGL::GL)