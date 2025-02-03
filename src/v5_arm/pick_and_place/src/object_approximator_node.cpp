#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <Eigen/Geometry>
#include <std_msgs/msg/string.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/pca.h>
#include <pcl/surface/convex_hull.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometric_shapes/shape_operations.h>
#include <pick_place_msgs/srv/add_scene_object.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>

class ObjectApproximatorNode : public rclcpp::Node
{
public:
    ObjectApproximatorNode() : Node("object_approximator")
    {
        add_grasp_object_service = this->create_service<pick_place_msgs::srv::AddSceneObject>(
            "pick_place/add_grasp_object", std::bind(&ObjectApproximatorNode::add_grasp_object, this, std::placeholders::_1, std::placeholders::_2));
        add_surface_service = this->create_service<pick_place_msgs::srv::AddSceneObject>("pick_place/add_surface", std::bind(&ObjectApproximatorNode::add_surface, this, std::placeholders::_1, std::placeholders::_2));
        collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/object_approximator/sphere_marker", 10);
        initCameraModel();
    }

private:

    const double SPHERICAL_DEVIATION_THRESHOLD = 0.5;
    struct Point3D
    {
        double x;
        double y;
        double z;
    };

    struct Sphere
    {
        double center_x;
        double center_y;
        double center_z;
        double radius;
    };

    struct Sphere_Collision_Object {
        Eigen::Vector3f center;
        double radius;
    };

    cv::Mat map1,
        map2;

    struct Box
    {
        Eigen::Vector3f dimensions;
        Eigen::Vector3f translation;
        Eigen::Matrix3f rotation;
    };

    /**
     * @brief Computes a sphere that approximates the given point cloud.
     * 
     * This function takes a point cloud of type pcl::PointCloud<pcl::PointXYZ> and computes
     * a sphere that best approximates the distribution of points in the cloud.
     * 
     * @param point_cloud The input point cloud containing points of type pcl::PointXYZ.
     * @return Sphere A structure representing the computed sphere.
     */
    struct Sphere computeSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
    {
        size_t n = point_cloud->points.size();
        Eigen::MatrixXd M(n, 4);
        Eigen::VectorXd b(n);

        for (size_t i = 0; i < n; ++i)
        {
            const pcl::PointXYZ p = point_cloud->points[i];
            M(i, 0) = p.x;
            M(i, 1) = p.y;
            M(i, 2) = p.z;
            M(i, 3) = 1.0;
            b(i) = -(p.x * p.x + p.y * p.y + p.z * p.z);
        }
        
        // Solve the linear system using Least Squares
        Eigen::VectorXd p = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        
        // Extract sphere parameters
        double A = p(0), B = p(1), C = p(2), D = p(3);
        double x_c = -A / 2.0;
        double y_c = -B / 2.0;
        double z_c = -C / 2.0;
        double r = std::sqrt(x_c * x_c + y_c * y_c + z_c * z_c - D);
        z_c += r;
        return {x_c, y_c, z_c, r};
    }


    /**
     * @brief Calculates the mean deviation of points in a point cloud from the surface of a given sphere.
     * 
     * This function computes the average deviation of each point in the provided point cloud from the surface of the specified sphere.
     * The deviation for each point is the absolute difference between the distance from the point to the sphere's center and the sphere's radius.
     * 
     * @param sphere A struct representing the sphere with fields center_x, center_y, center_z, and radius.
     * @param point_cloud A point cloud containing points of type pcl::PointXYZ.
     * @return double The Mean squared error of the points from the sphere's surface.
     */
    double calculateDeviationFromSphere(struct Sphere sphere, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
    {
        double deviation = 0.0;
        for (size_t i = 0; i < point_cloud->points.size(); ++i)
        {
            const pcl::PointXYZ p = point_cloud->points[i];
            double distance = std::sqrt((p.x - sphere.center_x) * (p.x - sphere.center_x) +
                                        (p.y - sphere.center_y) * (p.y - sphere.center_y) +
                                        (p.z - sphere.center_z) * (p.z - sphere.center_z));
            deviation += (distance - sphere.radius) * (distance - sphere.radius);
            
        }
        return deviation;
    }

    void initCameraModel()
    {

        double fx = 6.1244213867187500e+02;
        double fy = 6.1212139892578125e+02;
        double cx = 6.3851049804687500e+02;
        double cy = 3.6623443603515625e+02;

        cv::Mat K = cv::Mat::eye(3, 3, cv::DataType<float>::type);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;

        cv::Mat D = cv::Mat(1, 8, cv::DataType<float>::type);

        D.at<float>(0, 0) = 0.2063539773225784;
        D.at<float>(0, 1) = -2.55633544921875;
        D.at<float>(0, 2) = 0.0003513785777613521;
        D.at<float>(0, 3) = -0.0002052536583505571;
        D.at<float>(0, 4) = 1.615918040275574;
        D.at<float>(0, 5) = 0.08677487820386887;
        D.at<float>(0, 6) = -2.35876727104187;
        D.at<float>(0, 7) = 1.526051759719849;

        cv::initUndistortRectifyMap(K, D, cv::Mat(), K, cv::Size(1280, 720), CV_32FC1, map1, map2);
    }

    /**
     * @brief Adds a grasp object to the planning scene.
     * 
     * This function processes a depth image to extract a point cloud representing the object to be grasped.
     * It then fits a sphere to the point cloud and calculates the bounding box dimensions of the object.
     * The function publishes the collision object and a fake surface to the planning scene.
     * 
     * @param request Shared pointer to the service request containing the depth image and frame ID.
     * @param response Shared pointer to the service response containing the name of the grasp object.
     */
    void add_grasp_object(const std::shared_ptr<pick_place_msgs::srv::AddSceneObject::Request> request,
                          std::shared_ptr<pick_place_msgs::srv::AddSceneObject::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Adding grasp object to the planning scene");
        sensor_msgs::msg::Image depth_image_msg = request->depth_image;
        std::string frame = request->frame_id.data;
        std::cout << "Frame: " << frame << std::endl;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;
        pcl::PointCloud<pcl::PointXYZ> object_cloud = projectPoints(depth_image);
        object_cloud = *filterClusters(object_cloud.makeShared());

        // TODO: Add a least squares sphere fitting algorithm
        RCLCPP_INFO(this->get_logger(), "Fitting sphere to object cloud *******************************");
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_ptr = object_cloud.makeShared();
        struct Sphere sphere = computeSphere(object_cloud_ptr);
        RCLCPP_INFO(this->get_logger(), "Sphere center: (%f, %f, %f)", sphere.center_x, sphere.center_y, sphere.center_z);
        double deviation = calculateDeviationFromSphere(sphere, object_cloud_ptr);
        RCLCPP_INFO(this->get_logger(), "Mean deviation from sphere: %f", deviation);
        RCLCPP_INFO(this->get_logger(), "Target Deviation: %f", SPHERICAL_DEVIATION_THRESHOLD);
        RCLCPP_INFO(this->get_logger(), "Sphere radius: %f", sphere.radius);
        RCLCPP_INFO(this->get_logger(), "*********************************");


        // create a marker for the potential sphere created
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header.frame_id = frame;
        sphere_marker.header.stamp = this->get_clock()->now();
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;
        sphere_marker.pose.position.x = sphere.center_x;
        sphere_marker.pose.position.y = sphere.center_y;    
        sphere_marker.pose.position.z = sphere.center_z;
        sphere_marker.scale.x = sphere.radius * 2;
        sphere_marker.scale.y = sphere.radius * 2;
        sphere_marker.scale.z = sphere.radius * 2;
        sphere_marker.color.a = 1.0;
        sphere_marker.color.r = 1.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 1.0;
        marker_publisher_->publish(sphere_marker);
        
        if (deviation < SPHERICAL_DEVIATION_THRESHOLD)
        {
            RCLCPP_INFO(this->get_logger(), "Object is spherical");
            Sphere_Collision_Object sphere_collision_object;
            sphere_collision_object.center = Eigen::Vector3f(sphere.center_x, sphere.center_y, sphere.center_z);
            sphere_collision_object.radius = (sphere.radius);
            publishSphericalCollisionObject(sphere_collision_object, frame);
            rclcpp::sleep_for(std::chrono::seconds(1));
            response->object_name = "grasp_target";

            return;
        }

        Box object_box = calculateBoxDimensions(object_cloud.makeShared());
        RCLCPP_INFO(this->get_logger(), "Box dimensions: (%f, %f, %f)", object_box.dimensions.x(), object_box.dimensions.y(), object_box.dimensions.z());
        publishCollisionObject(object_box, frame);
        Eigen::Quaternionf box_quat(object_box.rotation);
        Eigen::Vector3f global_dimensions = box_quat * object_box.dimensions;

        rclcpp::sleep_for(std::chrono::seconds(1));

        response->object_name = "grasp_target";
        return;
        
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr get_upper_edge(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float tolerance = 0.01f)
    {
        float max_z = -1000;

        for (int i = 0; i < cloud->points.size(); i++)
        {

            if (cloud->points[i].z > max_z)
            {
                max_z = cloud->points[i].z;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr top_points(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (cloud->points[i].z > max_z - tolerance)
            {
                top_points->push_back(cloud->points[i]);
            }
        }

        return top_points;
    }

    void add_surface_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::string frame)
    {
        // Plane segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.0175);

        float closest_distance = std::numeric_limits<float>::max();
        pcl::PointCloud<pcl::PointXYZ>::Ptr closest_plane(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr closest_coefficients(new pcl::ModelCoefficients);

        int i = 0, nr_points = (int)cloud_filtered->points.size();
        while (cloud_filtered->points.size() > 5)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Check if the plane is horizontal
            Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            Eigen::Vector3f vertical_axis(0.0, 0.0, 1.0);
            float angle = acos(normal.dot(vertical_axis) / (normal.norm() * vertical_axis.norm()));
            RCLCPP_INFO(this->get_logger(), "Angle: %f", angle);

            const float angle_threshold = 10.0f * (M_PI / 180.0f); // 10 degrees
            if (angle < angle_threshold && inliers->indices.size() > 100)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_plane(new pcl::PointCloud<pcl::PointXYZ>);

                // Extract the planar inliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*temp_plane);

                // Calculate the distance of the plane from the origin
                float distance = std::abs(coefficients->values[3] / normal.norm());
                // height
                float height = -temp_plane->points[0].z;

                if (distance < closest_distance)
                {
                    closest_distance = height;
                    closest_plane = temp_plane;
                    *closest_coefficients = *coefficients;
                    // break;
                }
            }

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setNegative(true);
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.filter(*cloud_filtered);
            i++;
        }

        if (closest_plane->points.size() > 0)
        {
            closest_plane = filterClusters(closest_plane);

            pcl::io::savePLYFileASCII("largest_plane.ply", *closest_plane);
            std::cout << "Closest horizontal plane found at distance: " << closest_distance << std::endl;

            Eigen::Vector3f normal(closest_coefficients->values[0], closest_coefficients->values[1], closest_coefficients->values[2]);
            Eigen::Vector3f vertical_axis(0.0, 0.0, 1.0);
            float yaw_angle = acos(vertical_axis.dot(normal));
            Eigen::Matrix3f rotation_matrix;

            // Rotate the plane to align with the global z-axis
            Eigen::Vector3f axis = vertical_axis.cross(normal);
            axis.normalize();
            Eigen::AngleAxisf rotation(yaw_angle, axis);
            pcl::transformPointCloud(*closest_plane, *closest_plane, Eigen::Affine3f(rotation));

            rotation_matrix = rotation.toRotationMatrix();

            // Step 2: Find the dimensions (bounding box)
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*closest_plane, min_pt, max_pt);

            float length = max_pt.x() - min_pt.x();
            float width = max_pt.y() - min_pt.y();
            float height = max_pt.z() - min_pt.z();

            std::cout << "Table dimensions: " << std::endl;
            std::cout << "Length: " << length << " m" << std::endl;
            std::cout << "Width: " << width << " m" << std::endl;
            std::cout << "Height: " << height << " m" << std::endl;

            Box surface_box;
            surface_box.dimensions = Eigen::Vector3f(length, width, 0.02);
            surface_box.translation = Eigen::Vector3f((max_pt.x() + min_pt.x()) / 2, (max_pt.y() + min_pt.y()) / 2, min_pt.z() - 0.01);
            surface_box.rotation = rotation_matrix;
            publishCollisionObject(surface_box, frame, "surface");
        }
        else
        {
            std::cerr << "No horizontal plane found." << std::endl;

            // placeholder solution, publish fake surface
        }
    }

    void add_surface(const std::shared_ptr<pick_place_msgs::srv::AddSceneObject::Request> request,
                     std::shared_ptr<pick_place_msgs::srv::AddSceneObject::Response> response)
    {
        sensor_msgs::msg::Image depth_image_msg = request->depth_image;
        std::string frame = request->frame_id.data;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;
        pcl::PointCloud<pcl::PointXYZ> cloud = projectPoints(depth_image, 0.0, 3.5);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud.makeShared();

        cloud_filtered = get_upper_edge(cloud_filtered, 0.05f); // * Tanay added upper edge height calculation

        visualization_msgs::msg::Marker upper_edge_marker;
        upper_edge_marker.header.frame_id = frame;
        upper_edge_marker.header.stamp = this->get_clock()->now();

        upper_edge_marker.type = visualization_msgs::msg::Marker::CUBE;
        upper_edge_marker.action = visualization_msgs::msg::Marker::ADD;
        

        Box surface_box = calculateBoxDimensions(cloud_filtered);
        cout << "Surface box dimensions: " << surface_box.dimensions.transpose() << endl;
        surface_box.dimensions = Eigen::Vector3f(surface_box.dimensions.x(), surface_box.dimensions.y(), 0.02);

        // ! Commented this out so that we don't actually add something to the moveit scene
        // publishCollisionObject(surface_box, frame, "surface");

        response->object_name = "surface";
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // Step 1: Perform Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // Set the distance tolerance (2 cm in this case)
        ec.setMinClusterSize(100);    // Minimum number of points that form a cluster
        ec.setMaxClusterSize(25000);  // Maximum number of points in a cluster
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;

        if (cluster_indices.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No clusters found.");
            return cloud;
        }

        // Step 2: Extract the desired cluster
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
        pcl::PointIndices::Ptr largest_cluster(new pcl::PointIndices(*it));
        bool first_cluster = true;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CentroidPoint<pcl::PointXYZ> centroid_calculator;
        for (const auto &indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));
            extract.setInputCloud(cloud);
            extract.setIndices(indices_ptr);
            extract.setNegative(false);
            extract.filter(*cluster);

            std::cout << "Cluster contains " << cluster->points.size() << " points." << std::endl;

            if (cluster->points.size() > largest_cluster->indices.size() || first_cluster)
            {
                first_cluster = false;
                largest_cluster->indices = indices.indices;
            }
        }

        // Extract the largest or closest cluster
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(largest_cluster);
        extract.setNegative(false); // Keep only the selected cluster
        extract.filter(*filtered_cloud);

        // Step 3: Use the filtered cloud
        std::cout << "Filtered cloud contains " << filtered_cloud->points.size() << " points." << std::endl;

        return filtered_cloud;
    }

    float roundToNearest90(float angle)
    {
        float roundedAngle = round(angle / (M_PI / 2)) * (M_PI / 2);
        return roundedAngle;
    }

    void publishSphericalCollisionObject(Sphere_Collision_Object sphere, std::string frame, std::string object_id = "grasp_target") {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame;
        collision_object.id = object_id;

        moveit_msgs::msg::CollisionObject collision_object_orig;
        collision_object_orig.header.frame_id = frame;
        collision_object_orig.id = object_id + "_orig";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[primitive.SPHERE_RADIUS] = sphere.radius;
        

        geometry_msgs::msg::Pose sphere_pose;
        sphere_pose.position.x = sphere.center.x();
        sphere_pose.position.y = sphere.center.y();
        sphere_pose.position.z = sphere.center.z();

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(sphere_pose);
        collision_object.operation = collision_object.ADD;

        collision_object_publisher_->publish(collision_object);

        RCLCPP_INFO(this->get_logger(), "Adding the Spherical collision object to the world");
    }

    void publishCollisionObject(Box object_box, std::string frame, std::string object_id = "grasp_target")
    {

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame;
        collision_object.id = object_id;

        moveit_msgs::msg::CollisionObject collision_object_orig;
        collision_object_orig.header.frame_id = frame;
        collision_object_orig.id = object_id + "_orig";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = object_box.dimensions.x();
        primitive.dimensions[1] = object_box.dimensions.y();
        primitive.dimensions[2] = object_box.dimensions.z();

        std::cout << "Box Dimensions: " << object_box.dimensions.transpose() << std::endl;

        Eigen::Quaternionf quaternion(object_box.rotation);
        Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
        Eigen::Vector3f org_euler_angles = rotation_matrix.eulerAngles(0, 1, 2);
        std::cout << "Original Euler Angles Degrees: Roll=" << org_euler_angles.x() * 180 / M_PI << ", Pitch=" << org_euler_angles.y() * 180 / M_PI << ", Yaw=" << org_euler_angles.z() * 180 / M_PI << std::endl;

        // Extract the global x, y, and z axes from the rotation matrix
        Eigen::Vector3f global_x = rotation_matrix.col(0);
        Eigen::Vector3f global_y = rotation_matrix.col(1);
        Eigen::Vector3f global_z = rotation_matrix.col(2);

        // Extract the Euler angles from the global x and y axes
        float roll = std::atan2(global_y.z(), global_z.z());
        float pitch = std::atan2(-global_x.z(), std::sqrt(global_x.x() * global_x.x() + global_x.y() * global_x.y()));

        // Preserve the yaw (z-axis rotation) directly from the rotation matrix
        float yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

        std::cout << "Euler Angles Degrees: Roll=" << roll * 180 / M_PI << ", Pitch=" << pitch * 180 / M_PI << ", Yaw=" << yaw * 180 / M_PI << std::endl;

        // Apply the 90-degree constraint on global x and y axes (roll and pitch)
        roll = roundToNearest90(roll);
        pitch = roundToNearest90(pitch);

        std::cout << "Corrected Euler Angles Degrees: Roll=" << roll * 180 / M_PI << ", Pitch=" << pitch * 180 / M_PI << ", Yaw=" << yaw * 180 / M_PI << std::endl;

        // Reconstruct the rotation matrix using the constrained angles
        Eigen::Matrix3f constrained_rotation_matrix;
        constrained_rotation_matrix = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                                      Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                                      Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
        Eigen::Quaternionf constrainedQuaternion(constrained_rotation_matrix);

        geometry_msgs::msg::Pose box_pose;
        geometry_msgs::msg::Pose orig_pose;
        box_pose.orientation.x = constrainedQuaternion.x();
        box_pose.orientation.y = constrainedQuaternion.y();
        box_pose.orientation.z = constrainedQuaternion.z();
        box_pose.orientation.w = constrainedQuaternion.w();

        orig_pose.orientation.x = quaternion.x();
        orig_pose.orientation.y = quaternion.y();
        orig_pose.orientation.z = quaternion.z();
        orig_pose.orientation.w = quaternion.w();

        box_pose.position.x = object_box.translation.x();
        box_pose.position.y = object_box.translation.y();
        box_pose.position.z = object_box.translation.z();

        orig_pose.position.x = object_box.translation.x();
        orig_pose.position.y = object_box.translation.y();
        orig_pose.position.z = object_box.translation.z();

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_object_publisher_->publish(collision_object);

        collision_object_orig.primitives.push_back(primitive);
        collision_object_orig.primitive_poses.push_back(orig_pose);
        collision_object_orig.operation = collision_object.ADD;

        // delay time
        // rclcpp::sleep_for(std::chrono::seconds(1));

        // collision_object_publisher_->publish(collision_object_orig);

        RCLCPP_INFO(this->get_logger(), "Adding the collision object to the world");
    }

    Box calculateBoxDimensions(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(object_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(0.5);
        sor.filter(*cloud_filtered);

        std::cout << "Filtered cloud contains " << cloud_filtered->points.size() << " points." << std::endl;

        // Find prinicipal axes
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cloud_filtered);
        Eigen::Vector3f mean = pca.getMean().head<3>();
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Transform the point cloud to align with the principal axes
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() = -mean; // Translate to the origin
        pcl::transformPointCloud(*cloud_filtered, *aligned_cloud, transform);
        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation.block<3, 3>(0, 0) = eigen_vectors.transpose();
        pcl::transformPointCloud(*aligned_cloud, *aligned_cloud, rotation);

        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D(*aligned_cloud, min_point, max_point);

        Eigen::Vector3f dimensions = max_point.getVector3fMap() - min_point.getVector3fMap();
        Eigen::Vector3f translation = mean;
        std::cout << "Translation: " << mean << std::endl;

        Box box;
        box.dimensions = dimensions;
        box.translation = translation;
        box.rotation = eigen_vectors;

        pcl::io::savePLYFileASCII("output_cloud.ply", *aligned_cloud); // Change the file name as needed

        std::cout << "Dimensions: " << dimensions.transpose() << std::endl;
        std::cout << "Translation Trans: " << translation.transpose() << std::endl;
        std::cout << "Rotation matrix: \n"
                  << eigen_vectors << std::endl;

        return box;
    }

    cv::Mat undistort_img(cv::Mat in)
    {
        cv::Mat out;
        cv::remap(in, out, map1, map2, cv::INTER_NEAREST);
        return out;
    }

    pcl::PointCloud<pcl::PointXYZ> projectPoints(cv::Mat masked_depth, double min_depth = 0.0, double max_depth = 5.0)
    {
        Eigen::Matrix<float, 4, 4> Projection = Eigen::Matrix<float, 4, 4>::Zero();
        Projection(0, 0) = 613.09;
        Projection(1, 1) = 612.97;
        Projection(0, 2) = 641.01;
        Projection(1, 2) = 364.73;
        Projection(2, 2) = 1.0;
        Projection(3, 3) = 1.0;

        Eigen::Matrix<float, 4, 4> projectionInverse = Projection.inverse();

        cv::Mat depthImage;
        cv::resize(masked_depth, depthImage, cv::Size(1280, 720), 0, 0, cv::INTER_NEAREST);
        depthImage = undistort_img(depthImage);

        cv::Mat depthFloat;
        depthImage.convertTo(depthFloat, CV_32F);

        pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Check if the image was loaded successfully
        if (depthFloat.empty())
        {
            std::cout << "Failed to load the depth image.\n";
            return *segmented_pointcloud;
        }

        std::vector<int> indices_array;

        int index = 0;
        float depthValue = 0.0;
        for (int u = 0; u < depthFloat.cols; u += 3)
        {
            for (int v = 0; v < depthFloat.rows; v += 3)
            {
                depthValue = depthFloat.at<float>(v, u) * 0.001;

                if (depthValue == 0.0 || depthValue > max_depth || depthValue < min_depth)
                    continue;

                // transform to 3D relative frame
                Eigen::Vector4f imagePoint(u, v, 1.0, 1 / depthValue);
                Eigen::Vector4f world_point = depthValue * projectionInverse * imagePoint;

                // add point to new point cloud piece
                pcl::PointXYZ newPoint;
                newPoint.x = world_point.z() - 0.03; // offset closer
                newPoint.y = -world_point.x();
                newPoint.z = -world_point.y() - 0.05; // offset down
                /*newPoint.r = rgbImage.at<cv::Vec3b>(v, u)[2];
                newPoint.g = rgbImage.at<cv::Vec3b>(v, u)[1];
                newPoint.b = rgbImage.at<cv::Vec3b>(v, u)[0];*/

                if (isnan(newPoint.x) || isnan(newPoint.y) || isnan(newPoint.z))
                    continue;

                segmented_pointcloud->push_back(newPoint);
            }
        }

        pcl::io::savePLYFileASCII("whole-cloud" + std::to_string(rand()) + ".ply", *segmented_pointcloud);

        return *segmented_pointcloud;
    }

    rclcpp::Service<pick_place_msgs::srv::AddSceneObject>::SharedPtr add_grasp_object_service;
    rclcpp::Service<pick_place_msgs::srv::AddSceneObject>::SharedPtr add_surface_service;
    rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pick_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectApproximatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}