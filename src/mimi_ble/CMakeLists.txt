cmake_minimum_required(VERSION 3.8)
project(mimi_ble)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(mimi_interfaces REQUIRED)
#find_package(sdbus-c++ REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDBUSPP REQUIRED sdbus-c++>=1)
include_directories(${SDBUSPP_INCLUDE_DIRS})
link_directories(${SDBUSPP_LIBRARY_DIRS})
add_definitions(${SDBUSPP_CFLAGS_OTHER})

add_executable(ble_node src/ble_node.cpp src/ble_uart_client.cpp)

#target_include_directories(ble_node PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#)

target_link_libraries(ble_node PUBLIC
        rclcpp::rclcpp
        #        std_msgs::std_msgs__rosidl_typesupport_cpp
        mimi_interfaces::mimi_interfaces__rosidl_typesupport_cpp
        ${SDBUSPP_LIBRARIES}
)

install(TARGETS
        ble_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
