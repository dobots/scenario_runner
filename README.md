# How to use the Behaviortree.cpp in ROS2

## 1. Install behaviortree.cpp:
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

## 2. Create a new workspace and ROS package

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
2. Create a ROS2 package called bt_demo:
```
cd ~/<your_ws>/src
ros2 pkg create --build-type ament_cmake <package_name>
```
Our package is called `bt_demo`. Build this new package:
```
cd ~/<your_ws>
colcon build
. install/setup.bash
```

## 3. Create the source files to check battery level

1. Create an xml file called batterycheck_tree.xml which describes the behaviour tree:
```
<root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="check_battery"/>
        </Sequence>
     </BehaviorTree>
 </root>
```


2. Create a new file called `batterycheck.cpp`:
```
//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

using namespace BT;

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
 
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

 
    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
        
    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/bt_demo/xml/batterycheck_tree.xml");
    
    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();
  
  rclcpp::shutdown();
  return 0;
}
```
Modify the path of the xml file to your workspace.


4. Modify the CMakelists.txt to include this source file and the behaviortree library. The CMakelists.txt file should look like the following:
```
cmake_minimum_required(VERSION 3.5)
project(bt_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

#Add the executable and name it so you can run your node using ros2 run
add_executable(bt_main src/batterycheck.cpp)
ament_target_dependencies(batterycheck rclcpp std_msgs behaviortree_cpp_v3)

#Finally, add the install(TARGETS..) section so ros2 run can find your executable
install(TARGETS batterycheck DESTINATION lib/${PROJECT_NAME})
  
ament_package()
```
5. Edit the package.xml to include the behaviortree library:
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bt_demo</name>
  <version>0.0.0</version>
  <description>behavior trees demo</description>
  <maintainer email="reka.hajnovics@gmail.com">reka</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>behaviortree_cpp_v3</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

6. Run check dependencies: `rosdep install -i --from-path src --rosdistro foxy -y`
7. Build the package:`colcon build`
8. Run the executable: `ros2 run bt_demo batterycheck`

![battery_check.png](https://github.com/dobots/scenario_runner/blob/main/img/batterycheck.png)

## Add logging functionality

1. Copy the file called batterycheck.cpp and rename it to batterycheck_logger.cpp. Include the logger header at the top of the file:
```
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
```
2. In the main function include the StdCoutLogger and print the tree recursively:
```
int main(int argc, char * argv[])
{
...
    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    printTreeRecursively(tree.rootNode());
...
}
```
3. Modify the CMakelists.txt and add the batterycheck_logger.cpp to it:
```
add_executable(batterycheck_logger src/batterycheck_logger.cpp)
ament_target_dependencies(batterycheck_logger rclcpp std_msgs behaviortree_cpp_v3)

install(TARGETS batterycheck_logger DESTINATION lib/${PROJECT_NAME})
```
4. Build the package: `colcon build`
5. Run this file: `ros2 run bt_demo batterycheck_logger`
![batterycheck_logger](https://github.com/dobots/scenario_runner/blob/main/img/batterycheck_logger.png)

