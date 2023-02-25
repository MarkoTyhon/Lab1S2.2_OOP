cmake_minimum_required(VERSION 3.0.0)
project(LAB_OOP VERSION 0.1.0)


file(GLOB SRC_FILES "src/*.cpp")
add_executable(LAB_OOP main.cpp ${SRC_FILES})
target_include_directories(LAB_OOP PUBLIC "include")


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
