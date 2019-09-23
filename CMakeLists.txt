cmake_minimum_required(VERSION 2.8)

project(socketcan-test)
add_executable(${PROJECT_NAME} socket_can_test.cpp)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} pthread)
