cmake_minimum_required(VERSION 3.8)
project(convex_decomposition)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Gather source files
file(GLOB_RECURSE CD_SOURCE_CODE src/ConvexDecomposition/*.cpp)

# Declare the executable
add_executable(${PROJECT_NAME} src/DecomposeSample.cpp ${CD_SOURCE_CODE})

# Link against dependencies (if needed)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Export package
ament_package()

