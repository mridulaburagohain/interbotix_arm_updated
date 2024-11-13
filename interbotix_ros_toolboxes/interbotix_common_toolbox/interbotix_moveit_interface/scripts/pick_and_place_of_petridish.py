#!/usr/bin/env python3

import signal
import sys
from threading import Thread
from geometry_msgs.msg import Pose, Quaternion
from interbotix_moveit_interface_msgs.srv import MoveItPlan
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from tf_transformations import quaternion_from_euler
import time

# pick and place position and orientation Constants
PICK_POSITION = Pose()
PICK_POSITION.position.x = 0.48
PICK_POSITION.position.y = 0.23
PICK_POSITION.position.z = 0.15
PICK_POSITION.orientation.x = 0.0
PICK_POSITION.orientation.y = 0.0
PICK_POSITION.orientation.z = 0.0
PICK_POSITION.orientation.w = 1.0

PLACE_POSITION = Pose()
PLACE_POSITION.position.x = 0.48
PLACE_POSITION.position.y = -0.23
PLACE_POSITION.position.z = 0.15
PLACE_POSITION.orientation.x = 0.0
PLACE_POSITION.orientation.y = 0.0
PLACE_POSITION.orientation.z = 0.0
PLACE_POSITION.orientation.w = 1.0

STABLE_POSITION = Pose()
STABLE_POSITION.position.x = 0.23
STABLE_POSITION.position.y = 0.01
STABLE_POSITION.position.z = 0.33
STABLE_POSITION.orientation.x = 0.0
STABLE_POSITION.orientation.y = 0.0
STABLE_POSITION.orientation.z = 0.0
STABLE_POSITION.orientation.w = 1.0

class MoveItPickPlace(Node):
    command: MoveItPlan.Request

    def __init__(self):
        super().__init__(node_name='moveit_pickplace')

        # Create the MoveIt planning service client
        self.moveit_planner = self.create_client(MoveItPlan, 'moveit_plan')
        self.clear_markers_service = self.create_client(Empty, 'clear_markers')
        # Wait for the MoveIt services
        if not (
            self.moveit_planner.wait_for_service(2.0) and
            self.clear_markers_service.wait_for_service(2.0)
        ):
            sys.exit(1)

        # Initialize the MoveItPlan command
        self.command = MoveItPlan.Request()
        self.command.cmd = MoveItPlan.Request.CMD_NONE
        self.position = Pose()

    def grasp_event(self):
        self.get_logger().info('Grasping...')
        self.command.cmd = MoveItPlan.Request.CMD_GRIPPER_GRASP
        self.srv_moveit_plan()

    def release_event(self):
        self.get_logger().info('Releasing...')
        self.command.cmd = MoveItPlan.Request.CMD_GRIPPER_RELEASE
        self.srv_moveit_plan()
        
    def plan_pose_event(self):        
        self.plan(MoveItPlan.Request.CMD_PLAN_POSE)
        
    def plan(self, plan_type: int):        
        self.start_position = None
        self.goal_position = None
        self.path = [] 
        self.command.cmd = plan_type
        self.srv_moveit_plan()    
        
    def execute_event(self):
        self.command.cmd = MoveItPlan.Request.CMD_EXECUTE
        self.srv_moveit_plan()
        
    def update_moveit_interface(self,planning_group):
        selected_group = planning_group                   
        self.command.cmd = MoveItPlan.Request.CMD_CHANGE_PLANNING_GROUP
        self.command.new_planning_group = selected_group
        self.srv_moveit_plan()
        
    def execute_pick_place_event(self):
        self.execute_move_to_location(STABLE_POSITION)
        time.sleep(2)
        self.execute_move_to_location(PICK_POSITION)  # Step 1: Move to Location
        self.update_moveit_interface("interbotix_gripper")
        self.grasp_event()  # Step 2: Grasp the Object
        time.sleep(2)  # Wait for grasping to complete
        self.update_moveit_interface("interbotix_arm")
        self.execute_move_to_location(PLACE_POSITION)
        self.update_moveit_interface("interbotix_gripper")
        self.release_event()  # Step 4: Release the Object
        time.sleep(2)
        self.update_moveit_interface("interbotix_arm")
        self.execute_move_to_location(STABLE_POSITION)
        time.sleep(2)

    def execute_move_to_location(self,pose):
        # Set position and orientation values        
        self.position = pose
        
        #pose.position.x = 0.3  # Example: Position X
        #pose.position.y = 0.0  # Example: Position Y
        #pose.position.z = 0.2  # Example: Position Z
        #roll = 0.0  # Example: Roll angle (in radians)
        #pitch = 0.0  # Example: Pitch angle (in radians)
        #yaw = 0.0  # Example: Yaw angle (in radians)
        # Convert the roll, pitch, yaw to quaternion orientation
        #qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        #pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        #self.command.ee_pose = self.position
        self.get_logger().info("Planning move to location...")
        self.plan_pose_event()
        self.execute_event()

    def srv_moveit_plan(self):
        # If an 'Execute' request was received, call the service with the 'Execute' command
        if self.command.cmd == MoveItPlan.Request.CMD_EXECUTE:       
            execute_future: rclpy.Future = self.moveit_planner.call_async(self.command)
            execute_future.add_done_callback(self.execute_done_callback)                                                 
        elif self.command.cmd == MoveItPlan.Request.CMD_CHANGE_PLANNING_GROUP:       
            change_group: rclpy.Future = self.moveit_planner.call_async(self.command)
            change_group.add_done_callback(self.group_change_done_callback)          
        elif self.command.cmd in [MoveItPlan.Request.CMD_GRIPPER_GRASP, MoveItPlan.Request.CMD_GRIPPER_RELEASE]:
            gripper_future: rclpy.Future = self.moveit_planner.call_async(self.command)
            gripper_future.add_done_callback(self.gripper_done_callback)           
        elif self.command.cmd is not MoveItPlan.Request.CMD_NONE:
            pose = self.position           
            self.get_logger().info((
                'Desired end-effector pose: '
                f'x[m]: {pose.position.x:.2f}, y[m]: {pose.position.y:.2f}, '
                f'z[m] {pose.position.z:.2f}. '
               
            ))
            self.command.ee_pose = pose
            planner_future: rclpy.Future = self.moveit_planner.call_async(self.command)
            planner_future.add_done_callback(self.planner_done_callback)       

    def planner_done_callback(self, planner_future: rclpy.Future):
        planner_resp: MoveItPlan.Response = planner_future.result()
        self.get_logger().info(planner_resp.msg.data)       
        if planner_resp.success:
           self.planned_path = [list(point.positions) for point in planner_resp.trajectory.joint_trajectory.points]
        else:
           self.get_logger().info("no path planned")                     
        self.command.cmd = MoveItPlan.Request.CMD_NONE
        
    def execute_done_callback(self, execute_future: rclpy.Future):
        execute_resp: MoveItPlan.Response = execute_future.result()
        self.get_logger().info(execute_resp.msg.data)                       
        self.command.cmd = MoveItPlan.Request.CMD_NONE
    
    def gripper_done_callback(self, gripper_future: rclpy.Future):
        gripper_resp: MoveItPlan.Response = gripper_future.result()
        self.get_logger().info(gripper_resp.msg.data)                          
        self.command.cmd = MoveItPlan.Request.CMD_NONE 

    def group_change_done_callback(self, group_future: rclpy.Future):
        group_resp: MoveItPlan.Response = group_future.result()
        self.get_logger().info(group_resp.msg.data)  
        self.command.cmd = MoveItPlan.Request.CMD_NONE

    def reset_event(self):
        self.command.cmd = MoveItPlan.Request.CMD_NONE
        self.get_logger().info("Resetting...")

def main():
    rclpy.init()
    moveit_pickplace = MoveItPickPlace()

    # Execute the pick and place operation
    moveit_pickplace.execute_pick_place_event()

    rclpy.spin(moveit_pickplace)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

