cmake_minimum_required(VERSION 3.5)
project(ivcon)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

# Declare the executable
add_executable(${PROJECT_NAME} src/ivcon.c)

# Link libraries
target_link_libraries(${PROJECT_NAME} m)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Include ament package
ament_package()

