# Scenario runner

The scenario runner makes it possible to create multiple different environments with static and dynamic objects through a GUI. In addition, it can automatically run these scenarios, and read-only the end result of the simulation.

The training of machine learning algorithms in simulation requires many different simulation environments with slight modifications. Hundreds of simulations needs to be started, which is highly time-consuming and a tedious task. Therefore, we have come up with the idea of creating a scenario runner, which would automate this process. In addition, it would also make the experiments reproducible, which is really important for debugging and developing.

The scenario runner consists of multiple parts. The scenario runner is based on behaviour trees. The required scenario’s behaviour tree is described through the visual interface. The visual interface exports an xml scenario description file. This file is read and executed by the c++ library, which can handle behaviour trees. (see image)
<p align="center">
<img src="https://github.com/dobots/scenario_runner/blob/main/img/overview.png" width = "700" /> 
</p>

## Structure
This repository contains the following folders:

**bt_demo :** Contains tutorials on how to create behaviour trees. It is based on the official tutorials of the behaviortree.CPP library. It is suggested to start with this folder and walk through the tutorials, before starting to create your own scenarios.

**img :** Contains the images used in the documentation

**pugixml-1.12 :** Is a small library used in the processing of xml files

**scenarios :**  This is a  ROS2 package containing all the source files needed to run scenarios. In addition, its readme has a detailed description about scenarios, and how to create your own.

## Install instructions for Ubuntu

 1. To work with behavior trees you are encouraged to install the following dependencies:
```
sudo apt-get install libzmq3-dev libboost-dev
```

2.  Then you need to install the ROS2 package: 
```
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3
```

For this tutorial we are using the `foxy` ROS-DISTRO.
[Source](https://github.com/BehaviorTree/BehaviorTree.CPP)

## Next steps

For further instructions on how to start the scenarios follow the Readme in the scenarios folder. For further instructions on how to run the BT (behavior tree) tutorials follow the Readme in the bt_demo folder.

**I would highly recommend to start with the BT tutorials! The scenarios folder includes external links to additional tutorials and blogs as well, which will help to deeply understand BT trees, and learn how to create your own scenarios or run the already available ones.**

