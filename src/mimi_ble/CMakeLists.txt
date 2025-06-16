cmake_minimum_required(VERSION 3.8)
project(mimi_ble)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(mimi_interfaces REQUIRED)

add_executable(ble_node src/ble_node.cpp)
target_link_libraries(ble_node PUBLIC
        rclcpp::rclcpp
        #        std_msgs::std_msgs__rosidl_typesupport_cpp
        mimi_interfaces::mimi_interfaces__rosidl_typesupport_cpp
)

install(TARGETS
        ble_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
