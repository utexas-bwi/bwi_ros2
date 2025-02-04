#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pick_place_msgs/srv/enable_scene_cloud.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class SceneCloudPublisherNode : public rclcpp::Node
{
public:
    SceneCloudPublisherNode() : Node("scene_cloud_publisher_node")
    {
        // Create the first service
        pointcloud_srv = this->create_service<pick_place_msgs::srv::EnableSceneCloud>(
            "pick_place/toggle_scene_cloud",
            std::bind(&SceneCloudPublisherNode::enablePointcloudCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "low_cam/points2",
            10,
            std::bind(&SceneCloudPublisherNode::pointcloudCallback, this, std::placeholders::_1)
        );

        pointcloud_republisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/scene_cloud",
            10
        );
    }

private:
    bool pointcloud_enabled = false;

    void enablePointcloudCallback(
        const std::shared_ptr<pick_place_msgs::srv::EnableSceneCloud::Request> request,
        const std::shared_ptr<pick_place_msgs::srv::EnableSceneCloud::Response> response)
    {
        pointcloud_enabled = request->cloud_enabled;
        std::cout << "Pointcloud enabled: " << pointcloud_enabled << std::endl;

        response->success = true;
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (pointcloud_enabled)
        {
            pointcloud_republisher->publish(*msg);
        }
        else {
            sensor_msgs::msg::PointCloud2 empty_cloud = emptyCloudMessage();
            pointcloud_republisher->publish(empty_cloud);
        }
    }

    sensor_msgs::msg::PointCloud2 emptyCloudMessage () {

        pcl::PointCloud<pcl::PointXYZ> cloud;
        
        // Define the point cloud with one point
        cloud.width = 1;  // Set the number of points to 1
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        // Set the coordinates of the single point
        pcl::PointXYZ& point = cloud.points[0];
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;  // Set z to the maximum depth value

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "low_depth_camera_link";  // Set the frame ID
        output.header.stamp = this->get_clock()->now();

        return output;
    }

    rclcpp::Service<pick_place_msgs::srv::EnableSceneCloud>::SharedPtr pointcloud_srv;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_republisher;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SceneCloudPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}