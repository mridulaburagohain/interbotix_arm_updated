# interbotix_moveit_interface_msgs

As outlined in the main ```README``` of the interbotix_ros_toolboxes package, this module facilitates the setup of command options that can be sent from the GUI for communication with the moveit_plan server. It also handles the response returned by the interbotix_moveit_interface module.

- The updated code expands the command types to include save_trajectory, retrace, repeat, and gripper commands. Additionally, the response message structure of the class was modified to incorporate saved trajectory data. The changes were made in the ```srv/MoveItPlan.srv``` file

