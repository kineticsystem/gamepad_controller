# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

# IMPORTANT: the code does not compile with C++20.
# See https://bytemeta.vip/repo/ros2/rclcpp/issues/1936
# A fix has been already applied on the next release of Humble.

cmake_minimum_required(VERSION 3.22.1)
project(gamepad VERSION 0.1.0 LANGUAGES CXX)
include(GNUInstallDirs)

# Enable output of compile commands during generation.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
    sensor_msgs
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wno-gnu-zero-variadic-macro-arguments)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Build gamepad app.

# CMake does not need headers listed here and it is generally recommended not
# to add them to avoid additional maintenance burdens.
# Unfortunately, QtCreator needs them, to display them in the class view,
# for easily navigating between class and header files (F4) and for automatic
# code formatting.
add_executable(
    gamepad_node
    src/gamepad_main.cpp
    src/gamepad_node.cpp
    include/gamepad/gamepad_node.hpp
)
target_include_directories(
    gamepad_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# This is used by the linked to find all includes required by the target.
ament_target_dependencies(
    gamepad_node
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_features(gamepad_node
    PUBLIC cxx_std_17
)

###############################################################################
# INSTALL


install(
  TARGETS
    gamepad_node
  DESTINATION lib/gamepad)

# Install directories
install(
  DIRECTORY
    config
    launch
  DESTINATION share/gamepad
)

ament_package()
