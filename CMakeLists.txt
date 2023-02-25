cmake_minimum_required(VERSION 3.0.0)
project(LAB_OOP VERSION 0.1.0)


file(GLOB SRC_FILES "src/*.cpp")
add_executable(LAB_OOP main.cpp ${SRC_FILES})
target_include_directories(LAB_OOP PUBLIC "include")

add_custom_target(copy_assets
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/recourses "${CMAKE_CURRENT_BINARY_DIR}/$(Configuration)")

add_dependencies(LAB_OOP copy_assets)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
