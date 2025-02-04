cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pick_place_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pick_place_exec_node src/pick_place_exec_node.cpp)
ament_target_dependencies(pick_place_exec_node moveit_task_constructor_core rclcpp pick_place_msgs)
target_include_directories(pick_place_exec_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_place_exec_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(object_approximator_node src/object_approximator_node.cpp)
ament_target_dependencies(object_approximator_node rclcpp std_msgs sensor_msgs cv_bridge image_transport
OpenCV moveit_core moveit_ros_planning_interface moveit_msgs PCL pick_place_msgs)

add_executable(scene_cloud_publisher_node src/scene_cloud_publisher.cpp)
ament_target_dependencies(scene_cloud_publisher_node rclcpp sensor_msgs pick_place_msgs std_srvs PCL pcl_conversions)

add_executable(update_tracked_objects_node src/update_tracked_objects_node.cpp)
ament_target_dependencies(update_tracked_objects_node rclcpp pick_place_msgs std_msgs moveit_core moveit_ros_planning_interface moveit_msgs std_srvs yaml-cpp tf2_ros)

target_link_libraries(update_tracked_objects_node yaml-cpp)

install(TARGETS pick_place_exec_node object_approximator_node scene_cloud_publisher_node update_tracked_objects_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  pick_and_place/pick_place_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
