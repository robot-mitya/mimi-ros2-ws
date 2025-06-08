cmake_minimum_required(VERSION 3.8)
project(mimi_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(talker src/publisher_lambda_function.cpp)
ament_target_dependencies(talker
        rclcpp
        std_msgs
)

add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener
        rclcpp
        std_msgs
)

add_executable(gamepad_node src/gamepad_node.cpp)
ament_target_dependencies(gamepad_node
        rclcpp
        std_msgs
        sensor_msgs
)

install(TARGETS
  talker
  listener
  gamepad_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
