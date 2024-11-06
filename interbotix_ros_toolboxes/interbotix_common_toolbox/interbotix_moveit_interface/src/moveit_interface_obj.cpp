// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "interbotix_moveit_interface/moveit_interface_obj.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace interbotix
{

InterbotixMoveItInterface::InterbotixMoveItInterface(
  rclcpp::Node::SharedPtr & node,
  const std::string & planning_group)
: node_(node), planning_group_(planning_group)
{ 
  auto exec_temp = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_temp->add_node(node_);
  std::thread([&exec_temp]() {exec_temp->spin();}).detach();

  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_,
    planning_group);
  joint_model_group = move_group->getCurrentState(2.0)->getJointModelGroup(planning_group);
  
  auto current_state = move_group->getCurrentState(2.0);
  if (current_state) {
    std::size_t num_joints = current_state->getVariableCount();
    RCLCPP_INFO(node_->get_logger(), "Size of joint values: %zu", num_joints);
   } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to retrieve current robot state.");
   } 
  addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  // Subscriber to joint states
  joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/vx300s/joint_states", 
    rclcpp::QoS(10), 
    std::bind(&InterbotixMoveItInterface::jointStateCallback, this, std::placeholders::_1)
  );
  
  srv_moveit_plan = node_->create_service<MoveItPlan>(
    "moveit_plan",
    std::bind(
      &interbotix::InterbotixMoveItInterface::moveit_planner,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  srv_clear_markers = node_->create_service<Empty>(
    "clear_markers",
    std::bind(
      &interbotix::InterbotixMoveItInterface::clear_markers,
      this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
    node_,
    move_group->getPlanningFrame(),
    "/moveit_visual_tools");
  visual_tools->deleteAllMarkers();
  text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 0.770;
  visual_tools->publishText(
    text_pose,
    "InterbotixMoveItInterface",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->trigger();

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(
    node_->get_logger(),
    "Reference frame: %s", move_group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(
    node_->get_logger(),
    "End effector link: %s", move_group->getEndEffectorLink().c_str());

  // Stop executor and remove node from it so exec_ can be used later
  exec_temp->cancel();
  exec_temp->remove_node(node_);
}

InterbotixMoveItInterface::~InterbotixMoveItInterface()
{
  delete joint_model_group;
}
void InterbotixMoveItInterface::addFloorCollisionObject() {
  auto const collision_object = [frame_id = move_group->getPlanningFrame()] {  
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "floor";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {1.5, 1.5, 0.02};  // Size of the floor

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.01; // Position slightly below the robot base

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(floor_pose);
    collision_object.operation = collision_object.ADD;
    
    return collision_object;
    }();    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void InterbotixMoveItInterface::addCameraHolderCollisionObject() {
    auto const collision_object = [frame_id = move_group->getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "l_shaped_camera_holder";

        // Define the base of the L-shaped holder
        shape_msgs::msg::SolidPrimitive base;
        base.type = base.BOX;
        base.dimensions = {0.21, 0.28, 0.03};  // Length, Width, Height of base

        geometry_msgs::msg::Pose base_pose;
        base_pose.orientation.w = 1.0;
        base_pose.position.x = 0.5;  // 0.4 m away from the manipulator
        base_pose.position.y = 0.3;   // Y position
        base_pose.position.z = 0.015;   // Z position

        collision_object.primitives.push_back(base);
        collision_object.primitive_poses.push_back(base_pose);

        // Define the vertical part of the L-shaped holder
        shape_msgs::msg::SolidPrimitive vertical_part;
        vertical_part.type = vertical_part.BOX;
        vertical_part.dimensions = {0.035, 0.035, 0.3};  // Length, Width, Height of vertical part (x, y, z)

        geometry_msgs::msg::Pose vertical_pose;
        vertical_pose.orientation.w = 1.0;
        vertical_pose.position.x = 0.5; // Same x position as base
        vertical_pose.position.y = 0.4; // Adjusted Y position
        vertical_pose.position.z = 0.18; // Height above the base

        collision_object.primitives.push_back(vertical_part);
        collision_object.primitive_poses.push_back(vertical_pose);        
        
        // Define the horizontal part of the C-shaped holder
        shape_msgs::msg::SolidPrimitive horizontal_part;
        horizontal_part.type = horizontal_part.BOX;
        horizontal_part.dimensions = {0.035, 0.14, 0.035};  // Length, Width, Height

        geometry_msgs::msg::Pose horizontal_pose;
        horizontal_pose.orientation.w = 1.0;
        horizontal_pose.position.x = 0.5;  // 0.5 m away from the manipulator
        horizontal_pose.position.y = 0.3125;   // Y position
        horizontal_pose.position.z = 0.3125; // Z position

        collision_object.primitives.push_back(horizontal_part);
        collision_object.primitive_poses.push_back(horizontal_pose);
        
        // Define the cylindrical camera
        shape_msgs::msg::SolidPrimitive camera;
        camera.type = camera.CYLINDER;
        camera.dimensions = { 0.072,0.02}; // Height, Radius of the camera

        geometry_msgs::msg::Pose camera_pose;
        camera_pose.orientation.w = 1.0;
        camera_pose.position.x = 0.5; // Same x position as the vertical part
        camera_pose.position.y = 0.2225; // Same y position as the vertical part
        camera_pose.position.z = 0.3205; // Height above the base

        collision_object.primitives.push_back(camera);
        collision_object.primitive_poses.push_back(camera_pose);

        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();    

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}

void InterbotixMoveItInterface::addCameraHolderCollisionObjectSecond() {
    auto const collision_object = [frame_id = move_group->getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "l_shaped_camera_holder_second";

        // Define the base of the L-shaped holder
        shape_msgs::msg::SolidPrimitive base;
        base.type = base.BOX;
        base.dimensions = {0.21, 0.28, 0.03};  // Length, Width, Height of base

        geometry_msgs::msg::Pose base_pose;
        base_pose.orientation.w = 1.0;
        base_pose.position.x = 0.5;  // 0.4 m away from the manipulator
        base_pose.position.y = -0.3;   // Y position
        base_pose.position.z = 0.015;   // Z position

        collision_object.primitives.push_back(base);
        collision_object.primitive_poses.push_back(base_pose);

        // Define the vertical part of the L-shaped holder
        shape_msgs::msg::SolidPrimitive vertical_part;
        vertical_part.type = vertical_part.BOX;
        vertical_part.dimensions = {0.035, 0.035, 0.3};  // Length, Width, Height of vertical part (x, y, z)

        geometry_msgs::msg::Pose vertical_pose;
        vertical_pose.orientation.w = 1.0;
        vertical_pose.position.x = 0.5; // Same x position as base
        vertical_pose.position.y = -0.4; // Adjusted Y position
        vertical_pose.position.z = 0.18; // Height above the base

        collision_object.primitives.push_back(vertical_part);
        collision_object.primitive_poses.push_back(vertical_pose);        
        
        // Define the horizontal part of the C-shaped holder
        shape_msgs::msg::SolidPrimitive horizontal_part;
        horizontal_part.type = horizontal_part.BOX;
        horizontal_part.dimensions = {0.035, 0.14, 0.035};  // Length, Width, Height

        geometry_msgs::msg::Pose horizontal_pose;
        horizontal_pose.orientation.w = 1.0;
        horizontal_pose.position.x = 0.5;  // 0.5 m away from the manipulator
        horizontal_pose.position.y = -0.3125;   // Y position
        horizontal_pose.position.z = 0.3125; // Z position

        collision_object.primitives.push_back(horizontal_part);
        collision_object.primitive_poses.push_back(horizontal_pose);
        
        // Define the cylindrical camera
        shape_msgs::msg::SolidPrimitive camera;
        camera.type = camera.CYLINDER;
        camera.dimensions = { 0.072,0.02}; // Height, Radius of the camera

        geometry_msgs::msg::Pose camera_pose;
        camera_pose.orientation.w = 1.0;
        camera_pose.position.x = 0.5; // Same x position as the vertical part
        camera_pose.position.y = -0.2225; // Same y position as the vertical part
        camera_pose.position.z = 0.3205; // Height above the base

        collision_object.primitives.push_back(camera);
        collision_object.primitive_poses.push_back(camera_pose);

        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();    

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);
}
  
bool InterbotixMoveItInterface::changePlanningGroup(const std::string & new_planning_group) {
  if (planning_group_ != new_planning_group) {
    planning_group_ = new_planning_group;              
    //move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    //node_,
    //planning_group_);
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node_,
    planning_group_);
    const auto& robot_model = move_group->getRobotModel();        
    joint_model_group = robot_model->getJointModelGroup(planning_group_);
    addFloorCollisionObject();
    addCameraHolderCollisionObject(); 
    addCameraHolderCollisionObjectSecond();
    RCLCPP_INFO(node_->get_logger(), "Planning group changed to: %s", new_planning_group.c_str());
    if (joint_model_group){
    return true;
    }else{
    return false;}
    }
    
    return false;
}

void InterbotixMoveItInterface::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{   
    current_joint_values_ = msg->position; // Store the latest joint values
    
}

bool InterbotixMoveItInterface::moveit_plan_joint_positions(
  const std::vector<double> joint_group_positions)
{ addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  visual_tools->deleteAllMarkers();
  move_group->setJointValueTarget(joint_group_positions);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);
  
  visual_tools->publishText(
    text_pose,
    "Joint Space Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_pose(const geometry_msgs::msg::Pose pose, moveit_msgs::msg::RobotTrajectory &trajectory)
{ addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  visual_tools->deleteAllMarkers();
  move_group->setPoseTarget(pose);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);
  trajectory = saved_plan.trajectory_;
  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Pose Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan success: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_position(double x, double y, double z,moveit_msgs::msg::RobotTrajectory &trajectory)
{ addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  visual_tools->deleteAllMarkers();
  move_group->setPositionTarget(x, y, z);
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  RCLCPP_INFO(
    node_->get_logger(),
    "Target: x, y, z: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
  
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);
  trajectory = saved_plan.trajectory_;
  
  RCLCPP_INFO(
    node_->get_logger(),
    "Path: number of waypoints: %zu",saved_plan.trajectory_.joint_trajectory.points.size() );
  RCLCPP_INFO(
    node_->get_logger(),
    "Path: number of joints: %zu",saved_plan.trajectory_.joint_trajectory.points[0].positions.size() );
    
  for (size_t i = 0; i < saved_plan.trajectory_.joint_trajectory.points.size(); ++i) {
    const auto& point = saved_plan.trajectory_.joint_trajectory.points[i];
    RCLCPP_INFO(
        node_->get_logger(),
        "Waypoint %zu: positions: [%f, %f, %f, %f, %f, %f]",
        i,
        point.positions[0], // Assuming you're interested in the first joint position
        point.positions[1], // Modify based on how many joints you have
        point.positions[2],
        point.positions[3],
        point.positions[4],
        point.positions[5]
         // If you have more than three joints, continue as needed
        );
  };
  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Position Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan success: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_plan_ee_orientation(
  const geometry_msgs::msg::Quaternion quat, moveit_msgs::msg::RobotTrajectory &trajectory)
{ addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  visual_tools->deleteAllMarkers();
  move_group->setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
  bool success = (move_group->plan(saved_plan) == MoveItErrorCode::SUCCESS);
  trajectory = saved_plan.trajectory_;
  geometry_msgs::msg::Pose pose;
  pose = moveit_get_ee_pose();
  pose.orientation = quat;
  visual_tools->publishAxisLabeled(pose, VT_FRAME_NAME);
  visual_tools->publishText(
    text_pose,
    "Orientation Goal",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishTrajectoryLine(
    saved_plan.trajectory_,
    joint_model_group);
  visual_tools->trigger();

  RCLCPP_INFO(
    node_->get_logger(),
    "Plan successful: %d", success);
  return success;
}

bool InterbotixMoveItInterface::moveit_grasp() {
    // Move to the predefined grasp position
    move_group->setNamedTarget("Grasp_petridish"); // Replace with the actual name in your SRDF
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    addFloorCollisionObject();
    addCameraHolderCollisionObject(); 
    addCameraHolderCollisionObjectSecond();
    bool success = (move_group->plan(grasp_plan) == MoveItErrorCode::SUCCESS);
    if (success) {
        move_group->execute(grasp_plan); // Execute the grasp plan
    }
    
    return success;
}

bool InterbotixMoveItInterface::moveit_release() {
    // Move to the predefined release position
    addFloorCollisionObject();
    addCameraHolderCollisionObject(); 
    addCameraHolderCollisionObjectSecond();
    move_group->setNamedTarget("Released"); // Replace with the actual name in your SRDF
    moveit::planning_interface::MoveGroupInterface::Plan release_plan;
    
    bool success = (move_group->plan(release_plan) == MoveItErrorCode::SUCCESS);
    if (success) {
        move_group->execute(release_plan); // Execute the release plan
    }
    
    return success;
}

bool InterbotixMoveItInterface::moveit_plan_cartesian_path(
  const std::vector<geometry_msgs::msg::Pose> waypoints)
{ addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group->computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);
  RCLCPP_INFO(
    node_->get_logger(),
    "Visualizing (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  visual_tools->deleteAllMarkers();
  visual_tools->publishText(
    text_pose,
    "Cartesian Path",
    rviz_visual_tools::WHITE,
    rviz_visual_tools::XLARGE);
  visual_tools->publishPath(
    waypoints,
    rviz_visual_tools::LIME_GREEN,
    rviz_visual_tools::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    visual_tools->publishAxisLabeled(
      waypoints[i],
      "pt" + std::to_string(i),
      rviz_visual_tools::SMALL);
  }
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  saved_plan = plan;
  saved_plan.trajectory_ = trajectory;

  // If a plan was found for over 90% of the waypoints...
  // consider that a successful planning attempt
  return (1.0 - fraction < 0.1) && (fraction != -1.0);
}

bool InterbotixMoveItInterface::moveit_execute_plan(void)
{ addFloorCollisionObject();
  addCameraHolderCollisionObject(); 
  addCameraHolderCollisionObjectSecond();
  return move_group->execute(saved_plan) == MoveItErrorCode::SUCCESS;
  
}

bool InterbotixMoveItInterface::moveit_execute_reverse_plan(void)
{   addFloorCollisionObject();
    addCameraHolderCollisionObject(); 
    addCameraHolderCollisionObjectSecond();
    // Check if there's a saved plan to reverse
    if (saved_plan.trajectory_.joint_trajectory.points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "No saved trajectory to reverse.");
        return false; 
    }                    
    
    // Create a new trajectory for the reverse path
    moveit_msgs::msg::RobotTrajectory reversed_trajectory = saved_plan.trajectory_;
    
    RCLCPP_INFO(
    node_->get_logger(),
    "size of joint_state value: %zu",current_joint_values_.size());

    // Replace the last waypoint with the current joint values
    if (!current_joint_values_.empty() &&
        current_joint_values_.size() >=6 && reversed_trajectory.joint_trajectory.points.back().positions.size() ==6) {
        RCLCPP_INFO(
        node_->get_logger(),
        "obtained from joint states");
        for (size_t i = 0; i < 6; ++i) {
        reversed_trajectory.joint_trajectory.points.back().positions[i] = current_joint_values_[i];
    }
    }else {
        RCLCPP_WARN(node_->get_logger(), "Current joint values are not compatible with the trajectory.");
    }
    
    // Reverse the joint trajectory waypoints
    std::reverse(reversed_trajectory.joint_trajectory.points.begin(),
                 reversed_trajectory.joint_trajectory.points.end());
                 
    RCLCPP_INFO(node_->get_logger(), "Current Joint Values from joint_states: [%f, %f, %f, %f, %f, %f]", current_joint_values_[0],       current_joint_values_[1], current_joint_values_[2],current_joint_values_[3],current_joint_values_[4],current_joint_values_[5]);
    RCLCPP_INFO(node_->get_logger(), "First Waypoint from reversed: [%f, %f, %f, %f, %f, %f]",  reversed_trajectory.joint_trajectory.points.front().positions[0], reversed_trajectory.joint_trajectory.points.front().positions[1], reversed_trajectory.joint_trajectory.points.front().positions[2], reversed_trajectory.joint_trajectory.points.front().positions[3], reversed_trajectory.joint_trajectory.points.front().positions[4], reversed_trajectory.joint_trajectory.points.front().positions[5]);
    RCLCPP_INFO(node_->get_logger(), "saved path last point: [%f, %f, %f, %f, %f, %f]",  saved_plan.trajectory_.joint_trajectory.points.back().positions[0], saved_plan.trajectory_.joint_trajectory.points.back().positions[1], saved_plan.trajectory_.joint_trajectory.points.back().positions[2], saved_plan.trajectory_.joint_trajectory.points.back().positions[3], saved_plan.trajectory_.joint_trajectory.points.back().positions[4], saved_plan.trajectory_.joint_trajectory.points.back().positions[5]);             
    double time_from_start = 0.0;
    for (auto &point : reversed_trajectory.joint_trajectory.points) {
        point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
        time_from_start += 0.1;  // Adjust this value based on your needs
    }            
                  
    //move_group->setJointValueTarget(saved_plan.trajectory_.joint_trajectory.points.back().positions);
    move_group->setJointValueTarget(reversed_trajectory.joint_trajectory.points.front().positions);                
    // Execute the reversed plan
    //return move_group->execute(reversed_trajectory) == MoveItErrorCode::SUCCESS;
    auto result = move_group->execute(reversed_trajectory);
    if (result != MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed with error code: %d", static_cast<int>(result.val));
      }   
    return result == MoveItErrorCode::SUCCESS;
}

bool InterbotixMoveItInterface::moveit_execute_repeat_plan(void) 
{   addFloorCollisionObject();  
    addCameraHolderCollisionObject(); 
    addCameraHolderCollisionObjectSecond();
    // Replace the last waypoint with the current joint values
    if (!current_joint_values_.empty() &&
        current_joint_values_.size() >=6 && saved_plan.trajectory_.joint_trajectory.points.front().positions.size() ==6) {
        RCLCPP_INFO(
        node_->get_logger(),
        "obtained from joint states");
        for (size_t i = 0; i < 6; ++i) {
        saved_plan.trajectory_.joint_trajectory.points.front().positions[i] = current_joint_values_[i];
    }
    }else {
        RCLCPP_WARN(node_->get_logger(), "Current joint values are not compatible with the trajectory.");
    }
 
    move_group->setJointValueTarget(saved_plan.trajectory_.joint_trajectory.points.front().positions);                
   
    auto result = move_group->execute(saved_plan);
    if (result != MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed with error code: %d", static_cast<int>(result.val));
      }   
    return result == MoveItErrorCode::SUCCESS;
}

void InterbotixMoveItInterface::moveit_set_path_constraint(
  const std::string constrained_link,
  const std::string reference_link,
  const geometry_msgs::msg::Quaternion quat,
  const double tolerance)
{
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = constrained_link;
  ocm.header.frame_id = reference_link;
  ocm.orientation = quat;
  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;

  // this parameter sets the importance of this constraint relative to other constraints that might
  // be present. Closer to '0' means less important.
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group->setPathConstraints(test_constraints);

  // Since there is a constraint, it might take the planner a lot longer to come up with a valid
  // plan - so give it some time
  move_group->setPlanningTime(30.0);
}

void InterbotixMoveItInterface::moveit_clear_path_constraints(void)
{
  move_group->clearPathConstraints();

  // Now that there are no constraints, reduce the planning time to the default
  move_group->setPlanningTime(5.0);
}

geometry_msgs::msg::Pose InterbotixMoveItInterface::moveit_get_ee_pose(void)
{
  return move_group->getCurrentPose().pose;
}

void InterbotixMoveItInterface::moveit_scale_ee_velocity(const double factor)
{
  move_group->setMaxVelocityScalingFactor(factor);
}

bool InterbotixMoveItInterface::moveit_planner(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<MoveItPlan::Request> req,
  std::shared_ptr<MoveItPlan::Response> res)
{
  (void)request_header;
  bool success = false;
  std::string service_type;
  if (req->cmd == MoveItPlan::Request::CMD_CHANGE_PLANNING_GROUP) {
    success=changePlanningGroup(req->new_planning_group);
    service_type = "changed planning group";
  }else if (req->cmd == MoveItPlan::Request::CMD_PLAN_POSE) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    success = moveit_plan_ee_pose(req->ee_pose,trajectory);
    service_type = "Planning EE pose";
    res->trajectory = trajectory;
  } else if (req->cmd == MoveItPlan::Request::CMD_PLAN_POSITION) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    success = moveit_plan_ee_position(
      req->ee_pose.position.x,
      req->ee_pose.position.y,
      req->ee_pose.position.z, trajectory);
    service_type = "Planning EE position";
    res->trajectory = trajectory;
  } else if (req->cmd == MoveItPlan::Request::CMD_PLAN_ORIENTATION) {
    moveit_msgs::msg::RobotTrajectory trajectory;
    success = moveit_plan_ee_orientation(req->ee_pose.orientation,trajectory);
    service_type = "Planning EE orientation";
    res->trajectory = trajectory;
  } else if (req->cmd == MoveItPlan::Request::CMD_EXECUTE) {
            success = moveit_execute_plan();
            service_type = "Execution";
  }else if (req->cmd == MoveItPlan::Request::CMD_EXECUTE_AGAIN) {
            success = moveit_execute_repeat_plan();
            service_type = "Repeat path Execution";
  }else if (req->cmd == MoveItPlan::Request::CMD_EXECUTE_REVERSE){
     success = moveit_execute_reverse_plan(); // Implement this function
     service_type = "Reverse Execution";  
  }    
  // New gripper commands
  else if (req->cmd == MoveItPlan::Request::CMD_GRIPPER_GRASP) {
    success = moveit_grasp(); // Implement this function for grasping
    service_type = "Grasping";
  } else if (req->cmd == MoveItPlan::Request::CMD_GRIPPER_RELEASE) {
    success = moveit_release(); // Implement this function for releasing
    service_type = "Releasing";
  }
  res->success = success;
  if (success) {
    res->msg.data = service_type + " was successful!";
  } else {
    res->msg.data = service_type + " was not successful.";
  }

  return true;
}

bool InterbotixMoveItInterface::clear_markers(
  std::shared_ptr<rmw_request_id_t> request_header,
  std::shared_ptr<Empty::Request> req,
  std::shared_ptr<Empty::Response> res)
{
  (void)request_header;
  (void)req;
  (void)res;
  visual_tools->deleteAllMarkers();
  visual_tools->trigger();
  RCLCPP_DEBUG(node_->get_logger(), "Cleared markers.");
  return true;
}

}  // namespace interbotix
