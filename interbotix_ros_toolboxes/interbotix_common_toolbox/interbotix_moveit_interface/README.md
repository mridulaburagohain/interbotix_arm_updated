# interbotix_moveit_interface
As discused in the main ```README``` file of the *interbotix_ros_toolboxes* package this module is reponsible for the GUI modification

The files modified for our application has the following additions:
- *moveit_interface_obj.hpp* - modification to add new functions and change response type of already exsisting functions to the moveit_interface class was made. 
- *moveit_interface_obj.cpp* - this file initializes the moveit_interface node and leverages the move_group functionalities. New functions were implemented to handle saving, retracting, and repeating trajectories, as well as switching planning groups upon request. Additional functions enable grasping and releasing objects. The response structure was modified to include trajectory data. A new function was implemented to reverse saved trajectories for the retract operation. Functions were also added to handle collision object addition through the moveit_planning interface for each environmental object.
- *moveit_interface.cpp* - the node initialization was modified to take planning_group as a parameter
- *moveit_interface_gui* - introduced new buttons with full synchronization across planning groups and with other functionalities. Callback functions were added to handle trajectory saving and the execution of retract and repeat operations.
- *pick_and_place_moveit.py* - new script for performing safe pick-and-place operations based on object location.
- *gripper_control.py* - added a script to control the gripper using interbotix_control.
- *saved_paths.txt* - contains saved demo trajectories

## Reference from the original repository for the package structure
Below is an overview and package structure as defined in the original repository for reference

## Overview
This package contains a small API modeled after the [Move Group C++ Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/482dc9db944c785870274c35223b4d06f2f0bc90/doc/move_group_interface/src/move_group_interface_tutorial.cpp) that allows a user to command desired end-effector poses to an Interbotix arm. It is not meant to be all-encompassing but rather should be viewed as a starting point for someone interested in creating their own MoveIt interface to interact with an arm. The package also contains a small GUI that can be used to pose the end-effector.

Finally, this package also contains a modified version of the [Move Group Python Interface Tutorial](https://github.com/ros-planning/moveit_tutorials/blob/482dc9db944c785870274c35223b4d06f2f0bc90/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py) script that can be used as a guide for those users who would like to interface with an Interbotix robot via the MoveIt Commander Python module.

## Nodes
The *interbotix_moveit_interface* nodes are described below:
- **moveit_interface** - a small C++ API that makes it easier for a user to command custom poses to the end-effector of an Interbotix arm; it uses MoveIt's planner behind the scenes to generate desired joint trajectories
- **moveit_interface_gui** - a GUI (modeled after the one in the *joint_state_publisher* package) that allows a user to enter in desired end-effector poses via text fields or sliders; it uses the **moveit_interface** API to plan and execute trajectories
- **moveit_python_interface** - a modified version of the script used in the [Move Group Python Interface](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) tutorial that is meant to work with an Interbotix arm; just press 'Enter' in the terminal to walk through the different steps

## Usage
This package is not meant to be used by itself but with any robot platform that contains an arm (like a standalone arm or a mobile manipulator). Refer to the example ROS packages by those robot platforms to see more info on how this package is used. These nodes are not located there to avoid code duplicity.
