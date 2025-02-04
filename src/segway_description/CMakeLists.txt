cmake_minimum_required(VERSION 3.5)
project(segway_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_cmake_test REQUIRED)

# Install directories
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

# Add ament package hooks if needed (commented out as it's not commonly used in ROS 2)
# ament_package_hooks(50.segway_description
#   SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Include testing for launch files (requires launch_testing or manual checks)
  # find_package(ament_cmake_launch_testing REQUIRED)
  # Uncomment below and add test scripts if needed
  # ament_add_launch_test(launch/test_example_launch.py)
endif()

# Finalize package
ament_package()

