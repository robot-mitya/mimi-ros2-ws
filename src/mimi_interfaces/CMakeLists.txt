cmake_minimum_required(VERSION 3.8)
project(mimi_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/cmd/DriveCmd.msg"
        "msg/cmd/ScrAnimCmd.msg"
        "msg/cmd/HeadlightsCmd.msg"
)

ament_package()
