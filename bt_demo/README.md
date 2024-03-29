# Behaviortree.cpp tutorial 
Behaviour trees can create very complex tasks composed of simple ones. In game development behaviour trees are extremely popular, because even a simple behaviour tree can make a non-player character look smart.

Behaviour trees can be graphically represented as trees, which makes it easy for humans to understand and debug. It consists of nodes which can be classified as:

- root,

- composite nodes,

- execution nodes.


<p align="center">
<img src="https://github.com/dobots/scenario_runner/blob/main/img/wiki_bt.png" width = "400" /> 
</p>

For each pair of nodes, the outgoing node is called the parent and the incoming node is called the child. The root has only one child and no parent. The control flow node has one parent and at least one child. The execution node, also called the leaf node has no child only a parent.

Source: [https://www.wikiwand.com/en/Behavior_tree_(artificial_intelligence,_robotics_and_control)](https://www.wikiwand.com/en/Behavior_tree_(artificial_intelligence,_robotics_and_control))


## 0. Install behaviortree.cpp:
If you haven't already installed the ROS2 behavior-trees library then you need to install it now to continue with the tutorials:

 1. To work with behavior trees you are encouraged to install the following dependencies:
 ```   
sudo apt-get install libzmq3-dev libboost-dev
```

2.  Then you need to install the ROS2 package:
```
sudo apt-get install ros-$ROS_DISTRO-behaviortree-cpp-v3
```
For this tutorial, we are using the `foxy` ROS-DISTRO.
[Source](https://github.com/BehaviorTree/BehaviorTree.CPP)

## 2. Create a new workspace or use your existing workspace

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

## 3. Create a ROS2 package called bt_demo

In the next step, we will create a new package called bt_demo in your workspace.
You can either clone our bt_demo package, read through the tutorials, and only execute the code or you can create your own bt_demo package, and walk through the tutorials step-by-step. I would recommend the second option. This way you can use our bt_demo package for help and debugging.

3. Create a ROS2 package called bt_demo:
```
cd ~/<your_ws>/src
ros2 pkg create --build-type ament_cmake bt_demo
```
Build this new package:
```
cd ~/<your_ws>
colcon build
. install/setup.bash
```

**NOTE: The rest of the tutorials is a copy of the official [behaviortree.CPP](https://www.behaviortree.dev/) tutorials. When we tried to follow these tutorials, we faced many issues and unclarity. Therefore, we have decided to provide additional information to help beginners to follow and run these codes.**

## 4. Read the theory
Before we right dive into it, I would recommend reading the introduction and the "learn the basics" section at the official site: [https://www.behaviortree.dev/](https://www.behaviortree.dev/)

## 3. Create the source files to check the battery level
In the next section, you will start with a simplified version of the first tutorial. 

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


2. Create a new file called `batterycheck.cpp`.
It should look like this file: https://github.com/dobots/scenario_runner/blob/main/bt_demo/src/batterycheck.cpp

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

## 4. Add logging functionality
In this section, we will learn how to use logging.

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

## 5. First tutorial of the official guide
[Source: https://www.behaviortree.dev/tutorial_01_first_tree/](https://www.behaviortree.dev/tutorial_01_first_tree/)

1. Copy the file called batterycheck_logger.cpp and rename it to t01_create_tree.cpp. Add the new functions under the battery check functions:
```
/ Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface
{
public:
    GripperInterface(): _open(true) {}

    NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return NodeStatus::SUCCESS;
    }

    NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return NodeStatus::SUCCESS;
    }

private:
    bool _open; // shared information
};
```
In the main function after the BehaviorTreeFactory initialization call these functions:
```
// The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));
```

4. Modify the CMakelists.txt and add the t01_create_tree.cpp to it:
```
add_executable(t01_create_tree src/t01_create_tree.cpp)
ament_target_dependencies(t01_create_tree rclcpp std_msgs behaviortree_cpp_v3)

install(TARGETS t01_create_tree DESTINATION lib/${PROJECT_NAME})
```
5. Build the package: `colcon build`
6. Run this file: `ros2 run bt_demo t01_create_tree`.

![t01_tree](https://github.com/dobots/scenario_runner/blob/main/img/t01.png)

## 6. Move the functions to a separate file
1. Create a new file called simple_functions.hpp in the bt_demo/src repository and include the functions here from the t01_create_tree.cpp:
```
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface
{
public:
    GripperInterface(): _open(true) {}

    NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return NodeStatus::SUCCESS;
    }

    NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return NodeStatus::SUCCESS;
    }

private:
    bool _open; // shared information
};
```

2. Copy the file called t01_create_tree.cpp and rename it to t01_create_tree_distr.cpp. 
Modify this new file by including the previous hpp file  and removing  the function descriptions. The final file should look like this:
https://github.com/dobots/scenario_runner/blob/main/bt_demo/src/t01_create_tree_distr.cpp

Thanks to the .hpp format, we don't need to create a .h and a .cpp file. Above all, we don't need to compile them in a way to create libraries. The original tutorial is using traditional .h files, but it makes it more complicated to set up. I would recommend this solution.

3. Include the new t01_create_tree_distr.cpp in the CMakelists.txt:

5. Build the package: `colcon build`
6. Run this file: `ros2 run bt_demo t01_create_tree_distr`
![t01_tree](https://github.com/dobots/scenario_runner/blob/main/img/t01.png)


## Tutorial 2: basic ports
1. Read through the official tutorial: [https://www.behaviortree.dev/tutorial_02_basic_ports/](https://www.behaviortree.dev/tutorial_02_basic_ports/)

2. Move inside the xml folder and create a new file called t02.xml. Open this file and copy the xml from the tutorial:
```
<root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <SaySomething     message="start thinking..." />
           <ThinkWhatToSay   text="{the_answer}"/>
           <SaySomething     message="{the_answer}" />
           <SaySomething2    message="SaySomething2 works too..." />
           <SaySomething2    message="{the_answer}" />
       </Sequence>
    </BehaviorTree>
</root>
```

3. Move into the src directory of this package and copy the file called t01_create_tree_distr.cpp, then rename it t02_basic_ports.cpp
From the main function replaces the nodes from the previous tutorial with the nodes from the tutorial:
```
...
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

    // SimpleActionNodes can not define their own method providedPorts().
    // We should pass a PortsList explicitly if we want the Action to 
    // be able to use getInput() or setOutput();
    PortsList say_something_ports = { InputPort<std::string>("message") };
    factory.registerSimpleAction("SaySomething2", SaySomethingSimple,             say_something_ports );


    auto tree = factory.createTreeFromFile("<path-to-your-xml>t02.xml");
    ...

```
Don't forget to update the path to the t02.xml using your own path.

4. Inside the src folder create a new file containing all the necessary functions and call it functions_t02.hpp:
```
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    SaySomething(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    { }

    // It is mandatory to define this static method.
    static PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { InputPort<std::string>("message") };
    }

    // As usual, you must override the virtual function tick()
    NodeStatus tick() override
    {
        Optional<std::string> msg = getInput<std::string>("message");
        // Check if optional is valid. If not, throw its error
        if (!msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", 
                                   msg.error() );
        }

        // use the method value() to extract the valid message.
        std::cout << "Robot says: " << msg.value() << std::endl;
        return NodeStatus::SUCCESS;
    }
};

// Simple function that return a NodeStatus
BT::NodeStatus SaySomethingSimple(BT::TreeNode& self)
{
  Optional<std::string> msg = self.getInput<std::string>("message");
  // Check if optional is valid. If not, throw its error
  if (!msg)
  {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  // use the method value() to extract the valid message.
  std::cout << "Robot says: " << msg.value() << std::endl;
  return NodeStatus::SUCCESS;
}

class ThinkWhatToSay : public SyncActionNode
{
  public:
    ThinkWhatToSay(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    static PortsList providedPorts()
    {
        return { OutputPort<std::string>("text") };
    }

    // This Action writes a value into the port "text"
    NodeStatus tick() override
    {
        // the output may change at each tick(). Here we keep it simple.
        setOutput("text", "The answer is 42" );
        return NodeStatus::SUCCESS;
    }
};

```
6. Open your cpp file again and modify the name of the .hpp file to this new file:
```
#include "functions_t02.hpp"
```

8. Include the new t02_basic_ports.cpp in the CMakelists.txt:
```
...
add_executable(t02_basic_ports src/t02_basic_ports.cpp)
ament_target_dependencies(t02_basic_ports rclcpp std_msgs behaviortree_cpp_v3)

install(TARGETS t02_basic_ports DESTINATION lib/${PROJECT_NAME})
...
```

8. Navigate to the ROS2 workspace. Source the ros distribution, source this workspace and then build the package:
```
cd ~/<your_workspace>
source /opt/ros/foxy/setup.bash
. install/local_setup.bash
colcon build
```

9. Source it:`. install/local_setup.bash`

10. Run this file: `ros2 run bt_demo t02_basic_ports`

![t02_tree](https://github.com/dobots/scenario_runner/blob/main/img/t02.png)

![t02_flow](https://github.com/dobots/scenario_runner/blob/main/img/t02_flow.png)

# Tutorial 3: ports with generic types
1. Read through the official tutorial: [https://www.behaviortree.dev/tutorial_03_generic_ports/](https://www.behaviortree.dev/tutorial_03_generic_ports/)

2. Move inside the xml folder and create a new file called t02.xml. Open this file and copy the xml from the tutorial:
```
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <SequenceStar name="root">
            <CalculateGoal   goal="{GoalPosition}" />
            <PrintTarget     target="{GoalPosition}" />
            <SetBlackboard   output_key="OtherGoal" value="-1;3" />
            <PrintTarget     target="{OtherGoal}" />
        </SequenceStar>
     </BehaviorTree>
 </root>
```

3. Move into the src directory of this package and copy the file called t02_basic_ports.cpp. Rename it t03_generic_ports.
The main function replaces the nodes from the previous tutorial with the nodes from the tutorial:
```
...
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");


    auto tree = factory.createTreeFromFile("<path-to-your-xml>t03.xml");
    ...

```
Don't forget to update the path to the t03.xml using your own path.
Then you need to include the functions in the same file or you can create a separate .hpp file if you prefer. For simplicity, we will use the same file for this tutorial.
The final cpp file should look like this:
https://github.com/dobots/scenario_runner/blob/main/bt_demo/src/t03_generic_ports.cpp


4. Include the new t03_generic_ports.cpp in the CMakelists.txt:
```
...
add_executable(t03_generic_ports src/t03_generic_ports.cpp)
ament_target_dependencies(t03_generic_ports rclcpp std_msgs behaviortree_cpp_v3)

install(TARGETS t03_generic_ports DESTINATION lib/${PROJECT_NAME})
...
```

8. Navigate to the ROS2 workspace. Source the ros distribution, source this workspace and then build the package:
```
cd ~/<your_workspace>
source /opt/ros/foxy/setup.bash
. install/local_setup.bash
colcon build
```

9. Source it:`. install/local_setup.bash`

10. Run this file: `ros2 run bt_demo t03_generic_ports`

11. " The tree is a Sequence of 4 actions:
 1) Store a value of Position2D in the entry "GoalPosition"  using the action CalculateGoal.

2) Call PrintTarget. The input "target" will be read from the Blackboard entry "GoalPosition".

3) Use the built-in action SetBlackboard to write the key "OtherGoal".  Conversion from string to Position2D will be done under the hood.

4) Call PrintTarget. The input "goal" will be read from the Blackboard entry "OtherGoal"."

Quoted from: [https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/examples/t03_generic_ports.cpp](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/examples/t03_generic_ports.cpp)

>**Notice that there is a built-in action SetBlackboard which enables modification of its values and keys directly.**

# Tutorial 4: sequences

This tutorial explains the difference between a sequence node and a reactive sequence node.

I would recommend reading through the official tutorial:
https://www.behaviortree.dev/tutorial_04_sequence/

We will not replicate this tutorial, because it has multiple bugs, and dependencies and fixing them would be time-consuming.

Instead, we will modify the first tutorial to show the effect of using reactive nodes.

1. Copy the file t01.xml file and rename it to t04.xml. Open this file and modify this xml based on the tutorial to include a reactive sequence node:
```
  <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <ReactiveSequence>
           <CheckBattery/>
           <Sequence>
              <OpenGripper    name="open_gripper"/>
              <ApproachObject name="approach_object"/>
              <CloseGripper   name="close_gripper"/>
        </Sequence>
      </ReactiveSequence>  
     </BehaviorTree>
 </root>
```

2. Move into the src directory of this package and copy the file called t01_basic_ports.cpp. Rename it t04_create_tree_react.cpp.
Include a sleep function:
```
inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
```
Modify the ApproachObject function to change it **AsynActionNode** from **SyncActionNode** make it run longer by adding sleep:
```
class ApproachObject : public BT::AsyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::AsyncActionNode(name, {})
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        
        int count = 0;
        // Pretend that "computing" takes 250 milliseconds.
        while (count++ < 25)
        {
          SleepMS(10);
        }
                
        return BT::NodeStatus::SUCCESS;
    }
};  
```

Modify the .xml path to t04.xml. 
Comment the logger function.
Comment the `tree.tickRoot()`function and include the following print, which has multiple sleep functions:
```
NodeStatus status;

    std::cout << "\n--- 1st executeTick() ---" << std::endl;
    status = tree.tickRoot();

    SleepMS(150);
    std::cout << "\n--- 2nd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    SleepMS(150);
    std::cout << "\n--- 3rd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    std::cout << std::endl;
```

The final cpp file should look like this: https://github.com/dobots/scenario_runner/blob/main/bt_demo/src/t04_create_tree_react.cpp

4. Include the new t04_create_tree_react.cpp in the CMakelists.txt:
```
add_executable(t04_create_tree_react src/t04_create_tree_react.cpp)
ament_target_dependencies(t04_create_tree_react rclcpp std_msgs behaviortree_cpp_v3)

install(TARGETS t04_create_tree_react DESTINATION lib/${PROJECT_NAME})
```

8. Navigate to the ROS2 workspace. Source the ros distribution, source this workspace and then build the package:
```
cd ~/<your_workspace>
source /opt/ros/foxy/setup.bash
. install/local_setup.bash
colcon build
```

9. Source it:`. install/local_setup.bash`

10. Run this file: `ros2 run bt_demo t04_create_tree_react`. The output should look like this:
![t04_react](https://github.com/dobots/scenario_runner/blob/main/img/t04_react.png)

11. Now open the t04_create_tree_react.cpp file and modify the xml path to t01.xml. Rebuild, source, and then compare the output:
![t04_seq](https://github.com/dobots/scenario_runner/blob/main/img/t04_seq.png)

# Tutorial 5: subtrees

This tutorial explains how to create a complex tree from simple trees:

I would recommend reading through the official tutorial:
https://www.behaviortree.dev/tutorial_05_subtrees/

```
<root main_tree_to_execute = "MainTree">

    <BehaviorTree ID="DoorClosed">
        <Sequence name="door_closed_sequence">
            <Inverter>
                <IsDoorOpen/>
            </Inverter>
            <RetryUntilSuccesful num_attempts="4">
                <OpenDoor/>
            </RetryUntilSuccesful>
            <PassThroughDoor/>
        </Sequence>
    </BehaviorTree>

    <BehaviorTree ID="MainTree">
        <Fallback name="root_Fallback">
            <Sequence name="door_open_sequence">
                <IsDoorOpen/>
                <PassThroughDoor/>
            </Sequence>
            <SubTree ID="DoorClosed"/>
            <PassThroughWindow/>
        </Fallback>
    </BehaviorTree>

</root>
```

On the c++ side, we don't need to modify anything to use a tree composed of multiple trees. Therefore, this tutorial doesn't have any implementation. In addition, it introduces the loggers, which we already discussed at the beginning of the tutorials. 

# Tutorial 6-9: 

Most of these tutorials modify only the xml file, there is no c++ implementation and clearly explains the concept. Therefore, we will not provide any implementation either: https://www.behaviortree.dev/tutorial_06_subtree_ports/

## Groot visualization tool (Optional) 

[Groot](https://github.com/BehaviorTree/Groot) is used for creating, editing, and visualizing behavior trees. Installing it takes some effort, but if you would like to give it a try follow the next steps. It will not be needed to run the scenarios.

![groot](https://github.com/dobots/scenario_runner/blob/main/img/groot.png)

### Install Groot:
Install CMake from the software center:
https://vitux.com/how-to-install-cmake-on-ubuntu/

Install qt5 on ubuntu:
https://wiki.qt.io/Install_Qt_5_on_Ubuntu

Install dependencies:
```
sudo apt install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev
```

Follow the compilation instructions for Linux:
https://github.com/BehaviorTree/Groot
```
   git clone https://github.com/BehaviorTree/Groot.git
   cd Groot
   git submodule update --init --recursive
   mkdir build; cd build
   cmake ..
   make
```
### Start the application

Navigate into the build directory of Groot and start the executable from the command line:
```
cd ~/Groot/build
./Groot
```

To connect Groot and ROS you need to install ZeroMQ plugin. This is explained in the following tutorial:
https://medium.com/teamarimac/groot-with-ros-a8f7855f8e35

The sources to the packages in the tutorial are a little bit outdated, therefore I will provide links below to the newest packages:


Install dependencies:
```
sudo apt-get install libtool pkg-config build-essential autoconf automake
```

This is a new package:
```
sudo apt-get install libzmq3-dev
```

Install libsodium:
https://doc.libsodium.org/installation

Download a stable tarball of libsodium. I have used the following one: `libsodium-1.0.18-stable.tar.gz   `

Then execute the following commands:
```
./configure
make && make check
sudo make install
```

ZeroMQ 4.1.2. release will fail. Use therefore a newer version:

http://download.zeromq.org/

Download the `zeromq-4.1.4.tar.gz` tarball from the website. 

Then:
```
tar -xvf zeromq-4.1.4.tar.gz
cd zeromq-4.1.4
./autogen.sh
./configure && make check
sudo make install
sudo ldconfig
```

### Using Groot:

**1. Editor mode: load an existing xml into Groot:**
- you might get the error of custom nodes, then you need to register your nodes for Groot to  be able to visualize them:
https://navigation.ros.org/tutorials/docs/using_groot.html#adding-a-custom-node
- you also need to modify the xml for Groot:
https://www.behaviortree.dev/xml_format/



**2. Monitor the current tree:**

You need to include the PublisherZMQ into your main script and include its library:
```
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
...

int main(){
...
    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
    ...
    }
```

Then you can start your script e.g.:
```
ros2 run <your-package-name> <your-node-name>
```

Click in Groot to connect and it will show the current tree.

![groot_bt](https://github.com/dobots/scenario_runner/blob/main/img/groot_bt.png)
