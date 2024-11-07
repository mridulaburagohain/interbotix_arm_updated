# interbotix_moveit_interface_msgs

As discussed in the main ```README``` file of the interbotix_ros_toolboxes package, this module helps to set up the command options available to send from the GUI for communication and return response by the moveit_plan server used in the *interbotix_moveit_interface* module.

- The updated code expands the command types to include save_trajectory, retrace, repeat, and gripper commands. Additionally, the response message structure of the class was modified to incorporate saved trajectory data. The changes were made in the ```srv/MoveItPlan.srv``` file

