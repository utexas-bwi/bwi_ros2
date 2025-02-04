/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/generate_geometric_grasp.h>
#include <moveit/task_constructor/stages/generate_box_grasp.h>
#include <moveit/task_constructor/stages/generate_sphere_grasp.h>

#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("GenerateGeometricGrasp");

GenerateGeometricGrasp::GenerateGeometricGrasp(const std::string& name) : GeneratePose(name) {
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector");
	p.declare<std::string>("object");
	p.declare<double>("max_grasp_width");
	p.declare<double>("max_grasp_depth", 0.06);
	p.declare<double>("finger_length", 0.06);
	p.declare<boost::any>("pregrasp", "pregrasp posture");
	p.declare<boost::any>("grasp", "grasp posture");
}

static void applyPreGrasp(moveit::core::RobotState& state, const moveit::core::JointModelGroup* jmg,
                          const Property& diff_property) {
	try {
		// try named joint pose
		const std::string& diff_state_name{ boost::any_cast<std::string>(diff_property.value()) };
		if (!state.setToDefaultValues(jmg, diff_state_name)) {
			throw moveit::Exception{ "unknown state '" + diff_state_name + "'" };
		}
		return;
	} catch (const boost::bad_any_cast&) {
	}

	try {
		// try RobotState
		const moveit_msgs::msg::RobotState& robot_state_msg =
		    boost::any_cast<moveit_msgs::msg::RobotState>(diff_property.value());
		if (!robot_state_msg.is_diff)
			throw moveit::Exception{ "RobotState message must be a diff" };
		const auto& accepted = jmg->getJointModelNames();
		for (const auto& joint_name_list :
		     { robot_state_msg.joint_state.name, robot_state_msg.multi_dof_joint_state.joint_names })
			for (const auto& name : joint_name_list)
				if (std::find(accepted.cbegin(), accepted.cend(), name) == accepted.cend())
					throw moveit::Exception("joint '" + name + "' is not part of group '" + jmg->getName() + "'");
		robotStateMsgToRobotState(robot_state_msg, state);
		return;
	} catch (const boost::bad_any_cast&) {
	}

	throw moveit::Exception{ "no named pose or RobotState message" };
}

void GenerateGeometricGrasp::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GeneratePose::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check availability of object
	props.get<std::string>("object");
	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef)) {
		errors.push_back(*this, "unknown end effector: " + eef);
		throw errors;
	}

	// check availability of eef pose
	const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
	moveit::core::RobotState test_state{ robot_model };
	try {
		applyPreGrasp(test_state, jmg, props.property("pregrasp"));
	} catch (const moveit::Exception& e) {
		errors.push_back(*this, std::string{ "invalid pregrasp: " } + e.what());
	}

	if (errors)
		throw errors;
}

void GenerateGeometricGrasp::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	if (!scene->knowsFrameTransform(object)) {
		const std::string msg = "object '" + object + "' not in scene";
		spawn(InterfaceState{ scene }, SubTrajectory::failure(msg));
		return;
	}

	upstream_solutions_.push(&s);
}

void GenerateGeometricGrasp::compute() {
    if (upstream_solutions_.empty())
        return;
    
    planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

    // set end effector pose
    const auto& props = properties();
    const std::string& eef = props.get<std::string>("eef");
    const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

    moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();
    try {
        applyPreGrasp(robot_state, jmg, props.property("pregrasp"));
    } catch (const moveit::Exception& e) {
        spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "invalid pregrasp: " } + e.what()));
        return;
    }

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = props.get<std::string>("object");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject collision_object;
    auto collision_objects = planning_scene_interface.getObjects({props.get<std::string>("object")});

    double box_x, box_y, box_z;
    bool is_box = false;
    bool is_sphere = false;
    double sphere_radius = 0.0;

    if (!collision_objects.empty()) {
        collision_object = collision_objects.begin()->second;

        // Check the type of collision object and extract dimensions
        if (!collision_object.primitives.empty()) {
            for (const shape_msgs::msg::SolidPrimitive& primitive : collision_object.primitives) {
                if (primitive.type == shape_msgs::msg::SolidPrimitive::BOX) {
                    is_box = true;
                    box_x = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
                    box_y = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
                    box_z = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];

                    printf("Box dimensions: x = %f, y = %f, z = %f\n", box_x, box_y, box_z);
                    
                    GenerateGeometricGrasp::computeBox(scene, primitive);
                }
                else if (primitive.type == shape_msgs::msg::SolidPrimitive::SPHERE) {
                    is_sphere = true;
                    sphere_radius = primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];

                    printf("Sphere radius: %f\n", sphere_radius);
                    
                    GenerateGeometricGrasp::computeSphere(scene, primitive);
                }
            }
        }
        else {
            spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "Target object is not a box or sphere" }));
        }
    } else {
        spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "Target object not found" }));
    }
}

void GenerateGeometricGrasp::computeSphere(planning_scene::PlanningScenePtr scene, const shape_msgs::msg::SolidPrimitive& primitive) {

    const auto& props = properties();    

    geometry_msgs::msg::PoseStamped target_pose_msg;
	// ? To Carson: Does this need to match the object frame in moveit?
    target_pose_msg.header.frame_id = props.get<std::string>("object");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject collision_object;
	// ? Same thing here
    auto collision_objects = planning_scene_interface.getObjects({props.get<std::string>("object")});

	// Parameters from properties
    double finger_length = props.get<double>("finger_length");
    double hand_depth = props.get<double>("max_grasp_depth");

    // Extract sphere dimensions (radius)
    double sphere_radius = primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
	double z_distance = sphere_radius;
	
	if (finger_length > sphere_radius)
		z_distance = finger_length;

    std::cout << "Sphere radius: " << sphere_radius << std::endl;

	if (sphere_radius * 2 > props.get<double>("max_grasp_width"))
	{	
		spawn(InterfaceState{ scene }, SubTrajectory::failure(std::string{ "Sphere radius is too large" }));
		return;
	}

    Eigen::Isometry3d target_pose;
    
    // First, orbit around the x-axis (rotating around the X axis)
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
        target_pose = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
		//translate in relative to the target_pose's z after rotation
		Eigen::Vector3d relative_translation = target_pose.rotation() * Eigen::Vector3d(0, 0, z_distance);  // Translate in the Z-direction of the rotated frame
		target_pose.translation() = relative_translation;
        std::cout << "Pose after rotation around X-axis: " << target_pose.translation().transpose() << std::endl;
        addTargetPose(scene, target_pose, "Orbit around X", abs(angle) * 5);
    }

    // Then, orbit around the y-axis (rotating around the Y axis)
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
        target_pose = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
        Eigen::Vector3d relative_translation = target_pose.rotation() * Eigen::Vector3d(0, 0, z_distance);  // Translate in the Z-direction of the rotated frame
        target_pose.translation() = relative_translation;
		std::cout << "Pose after rotation around Y-axis: " << target_pose.translation().transpose() << std::endl;
        addTargetPose(scene, target_pose, "Orbit around Y", abs(angle) * 5);
    }

    // Finally, orbit around the z-axis (rotating around the Z axis)
    for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 4) {
        target_pose = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d relative_translation = target_pose.rotation() * Eigen::Vector3d(0, 0, z_distance);  // Translate in the Z-direction of the rotated frame
		target_pose.translation() = relative_translation;
        std::cout << "Pose after rotation around Z-axis: " << target_pose.translation().transpose() << std::endl;
        addTargetPose(scene, target_pose, "Orbit around Z", abs(angle) * 5);
    }
}


void GenerateGeometricGrasp::computeBox (planning_scene::PlanningScenePtr scene, const shape_msgs::msg::SolidPrimitive& primitive) {

    const auto& props = properties();    

    InterfaceState state(scene);
	geometry_msgs::msg::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = props.get<std::string>("object");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	moveit_msgs::msg::CollisionObject collision_object;
	auto collision_objects = planning_scene_interface.getObjects({props.get<std::string>("object")});

	double box_x, box_y, box_z;
    box_x = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
    box_y = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
    box_z = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];

    printf("Box dimensions: x = %f, y = %f, z = %f\n", box_x, box_y, box_z);

	double finger_length = props.get<double>("finger_length");
	double hand_depth = props.get<double>("max_grasp_depth");

	Eigen::Isometry3d target_pose;
	if (box_z < props.get<double>("max_grasp_width")) {
		if (box_y > box_x / 2) {
			for (double y_offset = -box_y / 5; y_offset <= box_y / 5; y_offset += 0.04) {  // 0.05 is an example increment
				double cost = abs(y_offset) * 5;
				double x_distance = box_x / 2;
				/*if (box_x < finger_length)
					x_distance += finger_length - box_x;*/
				//if (box_x > hand_depth)
				//x_distance -= std::min(hand_depth, box_x - hand_depth);
				x_distance -= hand_depth;
				target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
				target_pose *= Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
				target_pose.translation() = Eigen::Vector3d(x_distance, y_offset, 0);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z top", cost);
				target_pose = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());
				target_pose *= Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
				target_pose.translation() = Eigen::Vector3d(-x_distance, y_offset, 0);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z bottom", cost);
			}
		}
		if (box_x > box_y / 2) { 
			for (double x_offset = -box_x / 5; x_offset <= box_x / 5; x_offset += 0.04) {  // 0.05 is an example increment
				double cost = abs(x_offset) * 5;
				double y_distance = box_y / 2;
				/*if (box_y < finger_length)
					y_distance += finger_length - box_y;*/
				//if (box_y > hand_depth)
				//y_distance -= std::min(hand_depth, box_y - hand_depth);
				y_distance -= hand_depth;
				target_pose = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
				target_pose.translation() = Eigen::Vector3d(x_offset, y_distance, 0.0);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z side 1", cost);
				target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
				target_pose.translation() = Eigen::Vector3d(x_offset, -y_distance, 0.0);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z side 2", cost);
			}
		}

		double x_distance = hand_depth;
		/*if (box_x < finger_length)
			x_distance = finger_length - box_x;*/
		double y_distance = hand_depth;
		/*if (box_y < finger_length)
			y_distance = finger_length - box_y;*/

		target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
		target_pose *= Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(-box_x / 2 + x_distance, -box_y / 2 + y_distance, 0.0);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z corner 1", 0.0);
		target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
		target_pose *= Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(box_x / 2 - x_distance, -box_y / 2 + y_distance, 0.0);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z corner 2", 0.0);
		target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
		target_pose *= Eigen::AngleAxisd(-M_PI * (3.0/4), Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(-box_x / 2 + x_distance, box_y / 2 - y_distance, 0.0);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z corner 3", 0.0);
		target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
		target_pose *= Eigen::AngleAxisd(M_PI * (3.0/4), Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(box_x / 2 - x_distance, box_y / 2 - y_distance, 0.0);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Z corner 4", 0.0);
	}
	if (box_y < props.get<double>("max_grasp_width")) {
		if (box_z > box_x / 2) {
			for (double z_offset = -box_z / 4; z_offset <= box_z / 4; z_offset += 0.04) {  // 0.05 is an example increment
				double cost = abs(z_offset) * 5;
				double x_distance = box_x / 2;
				/*if (box_x < finger_length)
					x_distance += finger_length - box_x; */
				//if (box_x > hand_depth)
				//x_distance -= std::min(hand_depth, box_x - hand_depth);
				x_distance -= hand_depth;
				target_pose = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
				//target_pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
				target_pose.translation() = Eigen::Vector3d(x_distance, 0.0, z_offset);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y top", cost);
				target_pose = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY());
				//target_pose *= Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX());
				target_pose.translation() = Eigen::Vector3d(-x_distance, 0.0, z_offset);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y bottom", cost);
			}
		}
		if (box_x > box_z / 2) {
			for (double x_offset = -box_x / 4; x_offset <= box_x / 4; x_offset += 0.04) {  // 0.05 is an example increment
				double cost = abs(x_offset) * 5;
				double z_distance = box_z / 2;
				/*if (box_z < finger_length)
					z_distance += finger_length - box_z;*/
				//if (box_z > hand_depth)
				//z_distance -= std::min(hand_depth, box_z - hand_depth);
				z_distance -= hand_depth;
				target_pose = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
				target_pose.translation() = Eigen::Vector3d(x_offset, 0.0, z_distance);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y side 1", cost);
				target_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
				target_pose.translation() = Eigen::Vector3d(x_offset, 0.0, -z_distance);
				GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y side 2", cost);
			}
		}

		double x_distance = hand_depth;
		double y_distance = hand_depth;

		target_pose = Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(-box_x / 2 + x_distance, 0.0, box_z / 2 - y_distance);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y corner 1", 0.0);
		target_pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(box_x / 2 - x_distance, 0.0, box_z / 2 - y_distance);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y corner 2", 0.0);
		target_pose = Eigen::AngleAxisd(-M_PI * (3.0/4), Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(-box_x / 2 + x_distance, 0.0, -box_z / 2 + y_distance);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y corner 3", 0.0);
		target_pose = Eigen::AngleAxisd(M_PI * (3.0/4), Eigen::Vector3d::UnitY());
		target_pose.translation() = Eigen::Vector3d(box_x / 2 - x_distance, 0.0, -box_z / 2 + y_distance);
		GenerateGeometricGrasp::addTargetPose(scene, target_pose, "Y corner 4", 0.0);
	}
}

void GenerateGeometricGrasp::addTargetPose(planning_scene::PlanningScenePtr scene, const Eigen::Isometry3d& target_pose, const std::string& comment, const double cost) {
	InterfaceState state(scene);
	geometry_msgs::msg::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = properties().get<std::string>("object");;
	target_pose_msg.pose = tf2::toMsg(target_pose);
	state.properties().set("target_pose", target_pose_msg);

	SubTrajectory trajectory;
	trajectory.setCost(cost);
	trajectory.setComment(comment);

	// add frame at target pose
	rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

	spawn(std::move(state), std::move(trajectory));
}


}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
