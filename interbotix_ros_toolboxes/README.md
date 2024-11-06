# Interbotix_ros_toolboxes

This Repository is cloned from the Interbotix toolboxes repository and modified for our use for easy manipulation

#### Check the original codebase from interbotix: https://github.com/Interbotix/interbotix_ros_toolboxes/tree/humble
#### Original Documentation here: https://docs.trossenrobotics.com/interbotix_xsarms_docs/
## New Changes Included
Modules and files that were modified or newly added in the interbotix_ros_toolbox package includes:
```
├── interbotix_ros_toolboxes
│   └── interbotix_common_toolbox
│       ├── interbotix_moveit_interface
│           ├── src
│               ├── moveit_interface.cpp
│               └── moveit_interface_obj.cpp
│           ├── scripts
│               ├── moveit_interface_gui
│               ├── pick_and_place_moveit.py
│               └── gripper_control.py
│           ├── paths
│               └── saved_path.txt
│           └── include/interbotix_moveit_interface/moveit_interface_obj.hpp
│       └── interbotix_moveit_interface_msgs
│           ├── srv/MoveitPlan.srv
│           ├── CMakeLists.txt
│           └── package.xml
├── interbotix_ros_manipulators
│   └── ...
```
The addition and modification of new features to the moveit controlled  GUI is done by modifying the *interbotix_moveit_interface* and *interbotix_moveit_interface_msgs* modules using the above mentioned files. 
- **interbotix_moveit_interface** - This module is responsible to initialize to the moveGroupInterface, create services like moveit_plan and clear_markers and uses the GUI interpretations control the arm.
     - Previously the InterbotixMoveitInterface class was only able to control the arm without the gripper control and returns only a boolen value on any command execution call from the GUI.
     - While the updated code, has added a joint_states subscriber to check and update joint states before trajectory execution if an error threshold is surpassed, added collision objects using moveitPlanningScene to include environmental objects in its operation region, added the saved/obtained path as a msg response to the GUI call to be recorded for later use, added planning group change abilities during the control operation to enable gripper control with arm, added functions to retrace and repeat trajectories and gripper actions on GUI call.
     - The collision object created can serve as a base for future addition into the environment
- **interbotix_moveit_interface_msgs** - This module helps to set up the command options available to send from the GUI for communication and msg structure used to call and returned by the moveit_plan server used in the *interbotix_moveit_interface module*
     - The new code expanded the command types to include the save_trajectory, retrace, repeat and gripper commands along with adding saved trajectory in the response msg structure of the class.
 
Please find more details in the ```README``` file of the repective module
  
## Reference from the original repo for its structure  
Below is an overview and package structure as defined in the original repository for reference
## Overview
![toolbox_repo_structure](images/toolbox_repo_structure.png)

This repo contains support level ROS wrappers and robot interface modules that are used in other package for the interbotix arms.
Here this code is specifically for humble version of ROS but you want to use a different version do check out their original website.
Links to other repositories that use this repo include:
- [interbotix_ros_manipulators](https://github.com/mridulaburagohain/interbotix_arm_updated/tree/main/interbotix_ros_manipulators)

## Repo Structure
```
GitHub Landing Page: Explains repository structure and contains a single directory for each type of toolbox.
├── Toolbox Type X Landing Page: Contains support-level ROS packages for a given actuator/hardware platform.
│   ├── Support-Level Toolbox ROS Package 1
│   ├── Support-Level Toolbox ROS Package 2
│   └── Support-Level Toolbox ROS Package 3
│       ├── Robot Module Type 1
│       ├── Robot Module Type 2
│       └── Robot Module Type X
├── Support-Level Required Third Party Packages
│   ├── Third Party Package 1
│   ├── Third Party Package 2
│   └── Third Party Package X
├── LICENSE
└── README.md
```
As shown above, there are four main levels and two types of packages in this repository. To clarify some of the terms above, refer to the descriptions below.

- **Toolbox Type** - Toolboxes are broken up into types based on hardware or application. For example, one toolbox exists for Dynamixel-based robot platforms. Similarly, another toolbox exists for the Raspberry Pi platform. The Common toolbox on the other hand can be used for any application, regardless of hardware type. Future toolboxes could be based on other types of actuators or other computer platforms (like the Nvidia Jetson).

- **Support-Level Toolbox ROS Package** - This refers to a ROS package that is used for more than one Robot Type (like for manipulators and rovers). By putting the package here, there's only instance of the code instead of duplicates in multiple repositories. Some examples include the *interbotix_xs_ros_control* and *interbotix_moveit_interface* ROS packages as they are used both in the *interbotix_ros_manipulators* and *interbotix_ros_rovers* repositories.

- **Robot Module** - This refers to an interface module found in the *interbotix_XXXXX_modules* ROS package that builds on top of ROS using a more novice-friendly language like Python or MATLAB. Instead of writing a script using the various ROS libraries, one can simply import a module in whatever language they feel comfortable with and begin writing high-level programs. These modules are here because they can also be used for more than one robot type. For example, the *arm.py* module in the *interbotix_xs_modules* ROS package can be used both in X-Series LoCoBots found in the *interbotix_ros_rovers* repository and in the X-Series Arms found in the *interbotix_ros_manipulators* repository.

- **Support-Level Required Third Party Packages** - These packages are made by external organizations that the Support-Level Toolbox Packages require to run. Only packages that are not available on package indices like PyPI are stored here to reduce the git repository size. These packages will be managed using the git submodule feature. For example, the ModernRobotics Python library is available on PyPI, but the MATLAB library is not and is included here.




