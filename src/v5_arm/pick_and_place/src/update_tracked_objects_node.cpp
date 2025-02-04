#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <std_srvs/srv/empty.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

class UpdateTrackedObjectsNode : public rclcpp::Node
{
public:
    UpdateTrackedObjectsNode()
    : Node("update_tracked_objects_node"),
        tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(4))),
        tf_listener(*tf_buffer),
        tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
    {
        // Create a service to trigger object addition
        add_object_service_ = this->create_service<std_srvs::srv::Empty>(
            "pick_place/update_tracked_objects", 
            std::bind(&UpdateTrackedObjectsNode::add_objects, this, std::placeholders::_1, std::placeholders::_2));

        // Planning Scene Interface
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // YAML file containing object definitions
        objects_file_ = "/home/carson/v5_workspace/src/v5_arm/pick_and_place/data/objects.yaml";

        // Timer to update objects
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&UpdateTrackedObjectsNode::update_objects, this));

    }

private:
    void add_objects(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                     std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service called to add objects to the scene");
        update_objects();
        RCLCPP_INFO(this->get_logger(), "Service returned");
    }

    void update_objects()
    {
        RCLCPP_INFO(this->get_logger(), "Adding objects to scene");

        // Load YAML file containing objects
        YAML::Node objects;
        try {
            objects = YAML::LoadFile(objects_file_);
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            return;
        }

        //track unused objects
        std::vector<std::string> untracked_objects;

        // Iterate through objects in the YAML file
        for (const auto &object_entry : objects) {
            const std::string &object_name = object_entry.first.as<std::string>();
            const YAML::Node &object_data = object_entry.second;

            // Check if frame is present in tf
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer->lookupTransform(
                    "map", 
                    object_data["frame"].as<std::string>(), 
                    tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform for object %s: %s", object_name.c_str(), ex.what());
                untracked_objects.push_back(object_name);
                continue;
            }

            // Create a pose message for the object's frame
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = object_data["frame"].as<std::string>();

            // Define the orientation for z-up (default) or y-up configurations
            Eigen::Quaterniond q = Eigen::Quaterniond::Identity(); // Default z-up
            if (object_data["y_up"].as<bool>()) {
                q = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
                q = q * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
            }

            // Apply the rotation to the pose
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            // Transform the offset into the base frame
            Eigen::Vector3d offset(
                object_data["offset"]["x"].as<double>() / 2,
                object_data["offset"]["y"].as<double>() / 2,
                object_data["offset"]["z"].as<double>() / 2);

            Eigen::Vector3d transformed_offset = q * offset;

            pose.pose.position.x = transformed_offset.x();
            pose.pose.position.y = transformed_offset.y();
            pose.pose.position.z = transformed_offset.z();

            // Define the size of the object (box)
            const std::vector<double> size = {
                object_data["size"]["x"].as<double>(),
                object_data["size"]["y"].as<double>(),
                object_data["size"]["z"].as<double>()
            };

            // Create the collision object (box)
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = object_name;
            collision_object.header.frame_id = object_data["frame"].as<std::string>();

            // Define the box geometry
            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            primitive.dimensions = {size[0], size[1], size[2]};

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(pose.pose);
            collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

            // Apply the object to the planning scene
            planning_scene_interface_->applyCollisionObject(collision_object);

            RCLCPP_INFO(this->get_logger(), "Added object %s to the scene", object_name.c_str());
        }

        // Remove objects that are no longer tracked
        planning_scene_interface_->removeCollisionObjects(untracked_objects);

    }
    

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr add_object_service_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::string objects_file_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    tf2_ros::TransformListener tf_listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<UpdateTrackedObjectsNode>();

    // Spin the node to handle service requests and timer callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
