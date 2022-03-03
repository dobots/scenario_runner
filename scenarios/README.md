# Scenarios
The scenario runner makes it possible to create multiple different environments with static and dynamic objects. The scenario runner consists of multiple parts. The scenario runner is based on behaviour trees. The required scenario’s behaviour tree is described through the visual interface. The visual interface exports an xml scenario description file. This file is read and executed by the c++ library, which can handle behaviour trees. 

For the implementation of scenario runners the c++ library has been chosen, which can process behaviour trees described in an xml format. Source: [https://www.behaviortree.dev/bt_basics/](https://www.behaviortree.dev/bt_basics/)

One of the main reasons for choosing this library was that the Navigation2 package of ROS2 is based on this library as well.

“Nav2 uses behavior trees to call modular servers to complete an action. An action can be to compute a path, control effort, recovery, or any other navigation related action. These are each separate nodes that communicate with the behavior tree (BT) over a ROS action server. “
Source: https://navigation.ros.org/

By selecting the same library as the navigation library, we can reuse the nodes implemented by the navigation package in our scenarios. This will speed up the implementation of scenarios.

## Useful sources
To learn more about the concept of behavior trees I would suggest to read the following links:
[https://www.wikiwand.com/en/Behavior_tree_(artificial_intelligence,_robotics_and_control)](https://www.wikiwand.com/en/Behavior_tree_(artificial_intelligence,_robotics_and_control))
https://www.gamedeveloper.com/programming/behavior-trees-for-ai-how-they-work
[https://www.behaviortree.dev/bt_basics/](https://www.behaviortree.dev/bt_basics/)

To examine some real ROS2 examples I highly recommend visiting the repository of Adlinktech:
https://github.com/Adlink-ROS/BT_ros2

To learn how to write an action server and clients follow the ROS2 tutorial:
https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html

## Structure

This ROS2 package contains a src and an xml folder. 
**xml:** This folder contains the description of the scenarios in xml format
**src:** This folder contains the files, which read the xml description and execute the behavior tree. In addition in this file are the also the custom function definitions used in the xml file e.g. spawn_model.hpp

## Installation

### 0. Install behaviortree.cpp:
If you haven't already installed behavior-tree from the previous page, then do it now.

 1. On Ubuntu, you are encouraged to install the following dependencies:
```
sudo apt-get install libzmq3-dev libboost-dev
```

2.  ROS2 users: You can easily install the package with the command
```
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3
```
For this tutorial we are using the `foxy` ROS-DISTRO.
[Source](https://github.com/BehaviorTree/BehaviorTree.CPP)

### 1. Create a new workspace or use your existing workspace
1. Create a ROS2 workspace by following the official [ROS2 tutorials](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
```
source /opt/ros/foxy/setup.bash
mkdir -p ~/<your_ws>/src
cd ~/<your_ws>/src
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
. install/local_setup.bash
```
2. Or use your existing workspace for the next steps

### 2. Clone the scenarios package
In the next step we will  clone the scenarios package:
```
cd ~/<your_ws>/src
svn checkout https://github.com/dobots/scenario_runner/trunk/scenarios
```
You can also clone the whole repository to a local folder and then copy (or create a symbolic link from)  the scenarios folder to your ~/<your_ws>/ src folder.

Build this new package:
```
cd ~/<your_ws>
source /opt/ros/foxy/setup.bash
colcon build
. install/setup.bash
```

### 3. Run an example scenario
Open a terminal:
```
cd ~/<your_ws>
source /opt/ros/foxy/setup.bash
. install/setup.bash
```
Start Gazebo:
```
source /usr/share/gazebo/setup.sh
ros2 launch gazebo_ros gazebo.launch.py
```
If you don't have Gazebo installed, then please install it first.

Open a second terminal:
```
cd ~/<your_ws>
source /opt/ros/foxy/setup.bash
. install/setup.bash
```
Run the scenario to spawn multiple cylinders at a random position:
```
ros2 run scenarios spawn_multiple
```
![scenarios_random.png](https://github.com/dobots/scenario_runner/blob/main/img/scenario_random.png)


## Simple example

- xml
- src
- function
- Cmakelists
- package.xml


## Spawn model function


## Template to create your own custom functions







