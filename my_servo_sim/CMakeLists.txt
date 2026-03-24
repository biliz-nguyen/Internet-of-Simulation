cmake_minimum_required(VERSION 3.8)
project(my_servo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/Xacro files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install helper scripts
install(PROGRAMS
  scripts/serial_to_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
