cmake_minimum_required(VERSION 3.8)
project(docking_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/DrawingPath.msg"
  "msg/BatteryStatus.msg"
  "action/ExecuteDrawing.action"
  "action/DockRobot.action"
  "srv/SetRobotState.srv"
  DEPENDENCIES geometry_msgs
)

find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME}
)

ament_package()

