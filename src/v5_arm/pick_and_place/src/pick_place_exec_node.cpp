#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/bool.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/attached_body.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pick_place_msgs/srv/object_manipulation.hpp>
#include <std_srvs/srv/trigger.hpp>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void pick_object(std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Request> request, std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Response> response);
  void place_object(std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Request> request, std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Response> response);
  void handoff_object(std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  int doTask();
  mtc::Task pickTask(std::string obj, std::string surface);
  mtc::Task placeTask(std::string surface);
  mtc::Task handoff();
  Eigen::Isometry3d getSceneObjectPose(const std::string& object);
  void removeSceneObject(std::string object);

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask(std::string obj);
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<pick_place_msgs::srv::ObjectManipulation>::SharedPtr pick_service = node_->create_service<pick_place_msgs::srv::ObjectManipulation>(
    "pick_place/pick_scene_object", std::bind(&MTCTaskNode::pick_object, this, std::placeholders::_1, std::placeholders::_2));
  rclcpp::Service<pick_place_msgs::srv::ObjectManipulation>::SharedPtr place_service = node_->create_service<pick_place_msgs::srv::ObjectManipulation>(
    "pick_place/place_scene_object", std::bind(&MTCTaskNode::place_object, this, std::placeholders::_1, std::placeholders::_2));
  mtc::Stage* attach_object_stage;
  rclcpp::Service <std_srvs::srv::Trigger>::SharedPtr handoff_service = node_->create_service<std_srvs::srv::Trigger>(
    "/handoff_object", std::bind(&MTCTaskNode::handoff_object, this, std::placeholders::_1, std::placeholders::_2));

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor;
  Eigen::Quaterniond pick_orientation;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
  std_msgs::msg::String msg;
  msg.data = "grasp_target";

  tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description", tf_buffer);
  pick_orientation = Eigen::Quaterniond::Identity();
}

void MTCTaskNode::pick_object (std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Request> request, std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Response> response) {
  std::string obj = request->object_label;
  std::string surface = request->surface_label;
  task_ = pickTask(obj, surface);
  int result = doTask();
  response->success = result == 0;
  if (result == -1)
    response->error_message = "Failed to initialize task";
  else if (result == -2)
    response->error_message = "Failed to plan task";
  else if (result == -3)
    response->error_message = "Failed to execute task";
}

void MTCTaskNode::place_object (std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Request> request, std::shared_ptr<pick_place_msgs::srv::ObjectManipulation::Response> response) {
  std::string obj = request->object_label;
  std::string surface = request->surface_label;
  task_ = placeTask(surface);
  int result = doTask();
  response->success = result == 0;
  if (result == -1)
    response->error_message = "Failed to initialize task";
  else if (result == -2)
    response->error_message = "Failed to plan task";
  else if (result == -3)
    response->error_message = "Failed to execute task";
}

void MTCTaskNode::handoff_object (std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  task_ = handoff();
  int result = doTask();
  response->success = result == 0;
  //if (result == 0)
    //MTCTaskNode::removeSceneObject("grasp_target");
}

int MTCTaskNode::doTask()
{
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return -1;
  }

  if (!task_.plan(3))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return -2;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  RCLCPP_INFO_STREAM(LOGGER, "Publishing solution");

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return -3;
  }

  return 0;
}

mtc::Task MTCTaskNode::handoff() {
  mtc::Task task;
  task.stages()->setName("handoff task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "ur5_tool0";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto stage_allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage_allow_collision->allowCollisions("grasp_target",
                            task.getRobotModel()
                                ->getJointModelGroup(arm_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
  task.add(std::move(stage_allow_collision));
  auto stage_allow_collision2 = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
  stage_allow_collision2->allowCollisions("grasp_target",
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
  task.add(std::move(stage_allow_collision2));

  auto stage_move_to_handoff = std::make_unique<mtc::stages::MoveTo>("move to handoff", interpolation_planner);
  stage_move_to_handoff->setGroup(arm_group_name);
  stage_move_to_handoff->setGoal("Handoff");
  task.add(std::move(stage_move_to_handoff));

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_remove_obj =
          std::make_unique<mtc::stages::ModifyPlanningScene>("remove obj");
  stage_remove_obj->removeObject("grasp_target");
  task.add(std::move(stage_remove_obj));

  auto stage_move_to_home = std::make_unique<mtc::stages::MoveTo>("move to home", interpolation_planner);
  stage_move_to_home->setGroup(arm_group_name);
  stage_move_to_home->setGoal("home");
  task.add(std::move(stage_move_to_home));

  return task;

}

mtc::Task MTCTaskNode::pickTask(std::string obj, std::string surface)
{
  mtc::Task task;
  task.stages()->setName("pick task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "ur5_tool0";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.002);

  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  stage_move_to_pick->properties().set("max_distance", 0.01);
  stage_move_to_pick->properties().set("max_velocity_scaling_factor", 0.5);
  task.add(std::move(stage_move_to_pick));

  pick_orientation = getSceneObjectPose(obj).rotation();
  
  std:: cout << "Pick orientation 1: " << pick_orientation.w() << ", " << pick_orientation.x() << ", " << 
                  pick_orientation.y() << ", " << pick_orientation.z() << std::endl;

  attach_object_stage =
    nullptr;  // Forward attach_object_stage to place pose generator

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });

    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.15, 0.2);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    {
      // Sample grasp pose

      auto stage = std::make_unique<mtc::stages::GenerateGeometricGrasp>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(obj);
      stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      stage->setMaxGraspWidth(0.14);
      stage->setMaxGraspDepth(0.05);


      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.18;

        // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(obj,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (surface,object)");
      stage->allowCollisions(obj, surface, true);
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("closed");
      grasp->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(obj, hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.1);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("go to ready position", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("ready");
    stage->properties().set("max_velocity_scaling_factor", 0.5);  // 50% of maximum velocity
    stage->properties().set("max_acceleration_scaling_factor", 0.5);  // 50% of maximum acceleration
    task.add(std::move(stage));
  }

  return task;

}

mtc::Task MTCTaskNode::placeTask(std::string surface)
{

  std::string obj = "grasp_target";
  RCLCPP_INFO_STREAM(LOGGER, "Place task " << surface);

  mtc::Task task;
  task.stages()->setName("place task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "ur5_tool0";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.002); 

  // remove fake surface
  removeSceneObject("fake surface");

  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    stage_move_to_place->properties().set("max_distance", 0.01);
    stage_move_to_place->properties().set("max_velocity_scaling_factor", 0.5);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                         { "eef", "group", "ik_frame" });
  
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("drop object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.04, 0.1);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GenerateSurfacePlace>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(obj);
      stage->setHandFrame(hand_frame);
      stage->setSurface(surface);
      stage->setPlaceOrientation(pick_orientation);

      std::cout << "Pick orientation: " << pick_orientation.w() << ", " << pick_orientation.x() << ", " << pick_orientation.y() << ", " << pick_orientation.z() << std::endl;

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = obj;

      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(current_state_ptr);  // Hook into attach_object_stage
      
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      /*Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());*/
      //grasp_frame_transform.linear() = q.matrix();
      //grasp_frame_transform.translation().z() = 0.3;

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(obj);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(obj,
                            task.getRobotModel()
                                ->getJointModelGroup(hand_group_name)
                                ->getLinkModelNamesWithCollisionGeometry(),
                            false);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(obj, hand_frame);
      place->insert(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->properties().set("link", hand_frame);
      stage->setMinMaxDistance(0.1, 0.15);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    stage->properties().set("max_velocity_scaling_factor", 0.5);  // 50% of maximum velocity
    stage->properties().set("max_acceleration_scaling_factor", 0.5);  // 50% of maximum acceleration
    task.add(std::move(stage));
  }

  std::cout << "Task created" << std::endl;

  return task;
}

Eigen::Isometry3d MTCTaskNode::getSceneObjectPose(const std::string& object)
{
  planning_scene_monitor->startStateMonitor();
  planning_scene_monitor->requestPlanningSceneState();
  auto scene = planning_scene_monitor->getPlanningScene();
  auto object_pose = scene->getFrameTransform(object);
  return object_pose;
}

void MTCTaskNode::removeSceneObject(std::string object)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<std::string> object_ids;
  object_ids.push_back(object);
  planning_scene_interface.removeCollisionObjects(object_ids);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  //tf2_ros::Buffer tf_buffer(node_->get_clock());
  //tf2_ros::TransformListener tf_listener(tf_buffer);

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}