# This Project Need IFRA_LinkAttacher package dependency
https://github.com/IFRA-Cranfield/IFRA_LinkAttacher/tree/ros2



# OpenMANIPULATOR-X
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/openmanipulator_x/OpenManipulator.png">
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/openmanipulator_x/OpenManipulator_Chain_Capture.png" width="500">

The 4-DOF Open Manipulator-X now supports MoveIt 2, enabling enhanced motion planning and control for advanced robotic applications. This update also brings significant improvements to the teleoperation features, example use cases, and the graphical user interface (GUI), providing a more seamless and user-friendly experience for developers and researchers.

- Active Branches: noetic, humble, main
- Legacy Branches: *-devel

# ROBOTIS e-Manual for OpenMANIPULATOR-X
- [http://emanual.robotis.com/docs/en/platform/openmanipulator/](http://emanual.robotis.com/docs/en/platform/openmanipulator/)

# Open Source related to OpenMANIPULATOR-X
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_y](https://github.com/ROBOTIS-GIT/open_manipulator_y)
- [open_manipulator_p](https://github.com/ROBOTIS-GIT/open_manipulator_p)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
- [dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface)

# Documents and Videos related to OpenMANIPULATOR-X
- [ROBOTIS e-Manual for OpenMANIPULATOR-X](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for OpenMANIPULATOR-P](https://emanual.robotis.com/docs/en/platform/openmanipulator_p/overview/)
- [ROBOTIS e-Manual for DYNAMIXEL SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [ROBOTIS e-Manual for DYNAMIXEL Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [YouTube Play List for OpenMANIPULATOR](https://www.youtube.com/playlist?list=PLRG6WP3c31_WpEsB6_Rdt3KhiopXQlUkb)

cd ~/ros2_ws/src/
git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
git clone https://github.com/rorgot1/open_manipulator.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git

cd ~/ros2_ws && colcon build

cd ~/ros2_ws/src/ && git clone https://github.com/rorgot1/open_manipulator.git



sudo apt update && sudo apt install ros-$ROS_DISTRO-moveit
sudo apt update && sudo apt install ros-$ROS_DISTRO-gazebo-ros
sudo apt update && sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt update && sudo apt install ros-humble-moveit-servo

source install/local_setup.bash

        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'launch', 'open_manipulator_x_moveit_config', 'servo.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'open_manipulator_x_teleop', 'open_manipulator_x_teleop'],
            output='screen'
        ),
colcon build --packages-select \
    open_manipulator_x_description \
    linkattacher_msgs \
    open_manipulator_x_gui \
    open_manipulator_x_playground \
    open_manipulator_x_bringup \
    open_manipulator_x_moveit_config \
    open_manipulator_x_teleop \
    ros2_linkattacher \
    open_manipulator
        
    
