cmake_minimum_required(VERSION 3.8)
project(segbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and launch files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Adapted from ROS1 for Gazebo, meshes, robots, and URDF directories
install(DIRECTORY gazebo meshes robots urdf
  DESTINATION share/${PROJECT_NAME}
)

# Testing and linter configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Optional: Add URDF validation tests here
endif()

# ament package declaration
ament_package()
