/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Robert Haschke */

#include <moveit/task_constructor/stages/generate_surface_place.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>

#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/attached_body.h>

#include <Eigen/Geometry>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("GeneratePlacePose");

GenerateSurfacePlace::GenerateSurfacePlace(const std::string& name) : GeneratePose(name) {
	auto& p = properties();
	p.declare<std::string>("object");
	p.declare<std::string>("hand_frame");
	p.declare<std::string>("surface");
	p.declare<Eigen::Quaterniond>("place_orientation");
	p.declare<double>("sample_step", 0.05);
	p.declare<bool>("allow_z_flip", false, "allow placing objects upside down");
}

void GenerateSurfacePlace::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	bool frame_found = false;
	const moveit::core::LinkModel* link = nullptr;
	scene->getCurrentState().getFrameInfo(object, link, frame_found);
	std::string msg;
	if (!frame_found)
		msg = "frame '" + object + "' is not known";
	if (!link)
		msg = "frame '" + object + "' is not attached to the robot";
	if (!msg.empty()) {
		if (storeFailures()) {
			InterfaceState state(scene);
			SubTrajectory solution;
			solution.markAsFailure();
			solution.setComment(msg);
			spawn(std::move(state), std::move(solution));
		} else
			RCLCPP_WARN_STREAM(LOGGER, msg);
		return;
	}

	upstream_solutions_.push(&s);
}

void GenerateSurfacePlace::compute() {
	if (upstream_solutions_.empty())
		return;

	const SolutionBase& s = *upstream_solutions_.pop();
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene()->diff();
	const moveit::core::RobotState& robot_state = scene->getCurrentState();
	const auto& props = properties();

	const std::string& frame_id = props.get<std::string>("object");
	geometry_msgs::msg::PoseStamped ik_frame;
	ik_frame.header.frame_id = frame_id;
	ik_frame.pose = tf2::toMsg(Eigen::Isometry3d::Identity());

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	printf("Surface: %s\n", props.get<std::string>("surface").c_str());
	
	moveit_msgs::msg::CollisionObject collision_object;
	auto collision_objects = planning_scene_interface.getObjects({props.get<std::string>("surface")});

	double box_x, box_y, box_z;

	if (!collision_objects.empty())
	{
		collision_object = collision_objects.begin()->second;

		// Check the type of collision object and extract dimensions
		if (!collision_object.primitives.empty())
		{
			for (const shape_msgs::msg::SolidPrimitive& primitive : collision_object.primitives)
			{
				if (primitive.type == shape_msgs::msg::SolidPrimitive::BOX)
				{
					box_x = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
					box_y = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
					box_z = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
					

					printf("Box dimensions: x = %f, y = %f, z = %f\n", box_x, box_y, box_z);
				}
				// Add checks for other primitive types if needed (e.g., SPHERE, CYLINDER, CONE)
			}
		}
		else {
			spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "Target surface is not a box" }));
		}
	}
	else {
		spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "Target surface not found" }));
	}

	// sample poses in a grid above the target surface
	std::vector<geometry_msgs::msg::PoseStamped> poses;
	poses.reserve(100);
	for (double x = 0.12; x < box_x / 2; x += 0.1) {
		for (double y = 0.12; y < box_y / 2; y += 0.1) {
			geometry_msgs::msg::PoseStamped pose;
			pose.header.frame_id = props.get<std::string>("surface");
			pose.pose.position.x = x;
			pose.pose.position.y = y;
			pose.pose.position.z = box_z / 2 + 0.02;  // 2cm above surface

			// sample rotations around z-axis
			for (double z = -M_PI; z < M_PI; z += M_PI / 6.0) {
				Eigen::Quaterniond q(Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()));
				pose.pose.orientation.x = q.x();
				pose.pose.orientation.y = q.y();
				pose.pose.orientation.z = q.z();
				pose.pose.orientation.w = q.w();
				
				// Original (x, y)
				poses.push_back(pose);

				// -x, y
				pose.pose.position.x = -x;
				pose.pose.position.y = y;
				poses.push_back(pose);

				// x, -y
				pose.pose.position.x = x;
				pose.pose.position.y = -y;
				poses.push_back(pose);

				// -x, -y
				pose.pose.position.x = -x;
				pose.pose.position.y = -y;
				poses.push_back(pose);
			}
		}
	}

	for (const auto& pose : poses) {

		const moveit::core::AttachedBody* object = robot_state.getAttachedBody(frame_id);
		const geometry_msgs::msg::PoseStamped& pose_msg = pose;

		Eigen::Isometry3d target_pose;
		tf2::fromMsg(pose.pose, target_pose);
		// target pose w.r.t. planning frame
		scene->getTransforms().transformPose(pose_msg.header.frame_id, target_pose, target_pose);
		std::string hand_frame = props.get<std::string>("hand_frame");
		Eigen::Isometry3d hand_pose = robot_state.getGlobalLinkTransform(hand_frame);

		Eigen::Isometry3d object_pose = object->getGlobalPose();
		Eigen::Quaterniond object_orientation = props.get<Eigen::Quaterniond>("place_orientation");
		if (object_orientation == Eigen::Quaterniond::Identity())
			object_orientation = object_pose.rotation();

		target_pose = target_pose * object_orientation;
		const double* dims = static_cast<const shapes::Box&>(*object->getShapes()[0]).size;
		Eigen::Vector3d scale_local(dims[0], dims[1], dims[2]);
		//std::cout << "Size: " << dims[0] << " " << dims[1] << " " << dims[2] << std::endl;
		Eigen::Vector3d scale_global = object_orientation * scale_local;

		double z_offset = abs(scale_global.z() / 2);
		target_pose.translation().z() += z_offset;
		
		// add rotations around z-axis
		geometry_msgs::msg::PoseStamped target_pose_msg;
		target_pose_msg.header.frame_id = scene->getPlanningFrame();
		target_pose_msg.pose = tf2::toMsg(target_pose);
		InterfaceState state(scene);
		forwardProperties(*s.end(), state);
		state.properties().set("target_pose", target_pose_msg);
		state.properties().set("ik_frame", ik_frame);
		SubTrajectory trajectory;
		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "place frame");
		spawn(std::move(state), std::move(trajectory));
	}
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
