# Interbotix_ros_toolboxes

This Repository is cloned from the Interbotix toolboxes repository and modified for our use for easy manipulation

#### Check the original codebase from interbotix: https://github.com/Interbotix/interbotix_ros_toolboxes/tree/humble
#### Original Documentation here: https://docs.trossenrobotics.com/interbotix_xsarms_docs/

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





