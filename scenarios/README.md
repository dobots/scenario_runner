# Scenarios
The scenario runner makes it possible to create multiple different environments with static and dynamic objects. The scenario runner consists of multiple parts. It is based on behaviour trees. The required scenario’s behaviour tree is described through the visual interface. The visual interface exports an xml scenario description file. This file is read and executed by the c++ library, which can handle behaviour trees. 

For the implementation of scenario runners, the c++ library has been chosen, which can process behaviour trees described in an xml format. Source: [https://www.behaviortree.dev/bt_basics/](https://www.behaviortree.dev/bt_basics/)

One of the main reasons for choosing this library was that the Navigation2 package of ROS2 is based on this library as well.

“Nav2 uses behavior trees to call modular servers to complete an action. An action can be to compute a path, control effort, recovery, or any other navigation-related action. These are each separate nodes that communicate with the behavior tree (BT) over a ROS action server. “
Source: https://navigation.ros.org/

By selecting the same library as the navigation library, we can reuse the nodes implemented by the navigation package in our scenarios. This will speed up the implementation of scenarios.

## Useful sources
To learn more about the concept of behavior trees I would suggest reading through the following links:
[https://www.wikiwand.com/en/Behavior_tree_(artificial_intelligence,_robotics_and_control)](https://www.wikiwand.com/en/Behavior_tree_(artificial_intelligence,_robotics_and_control))

[https://www.gamedeveloper.com/programming/behavior-trees-for-ai-how-they-work](https://www.gamedeveloper.com/programming/behavior-trees-for-ai-how-they-work)

[https://www.behaviortree.dev/bt_basics/](https://www.behaviortree.dev/bt_basics/)

To examine a behavior tree using ROS2 nodes I highly recommend visiting the repository of Adlinktech:
https://github.com/Adlink-ROS/BT_ros2

To learn how to write an action server and clients follow the ROS2 tutorial:
https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html

## Structure

The scenarios ROS2 package contains a src and an xml folder. 

**xml:** This folder contains the description of the scenarios in xml format

**src:** This folder contains the files, which read the xml description and execute the behavior tree. In addition, in this folder you can find the custom function definitions used in the xml file e.g. spawn_model.hpp

## Installation

### 0. Install behaviortree.cpp:
If you haven't already installed the ROS2 behavior-trees library, then you need to install it now to be able to use the scenarios:

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
You can also clone the whole repository to a local folder and then copy (or create a symbolic link from)  the scenarios folder to your `~/<your_ws>/src` folder.

Build this new package:
```
cd ~/<your_ws>
source /opt/ros/foxy/setup.bash
colcon build
. install/setup.bash
```

## Run a scenario to spawn cylinders at random position
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

> In case of gazebo errors run: `sudo killall -9 gazebo gzserver gzclient`. Then try to launch the Gazebo world again.

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


## Code structure
In this section, we will walk through the structure of this ROS package and discuss in detail each component needed for creating a scenario.

**xml folder:** Contains the description of the behavior tree.

In the previous example we have started the `spawn_multiple.xml` file, which looks like the following:
```
<root main_tree_to_execute = "MainTree" >

<BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <Repeat num_cycles = "10">
            <SpawnModel   name="spawn_model" model_name="blue_cylinder"  x="rand" y="rand" z="rand" R="rand" P="rand" Y="rand" file_path="<path_to_the_model_folder>/model.sdf" />
          </Repeat>  
            <SpawnModel   name="spawn_model" model_name="red_cylinder" x="rand" y="rand" z="rand" R="rand" P="rand" Y="rand" file_path="<path_to_the_model_folder>/model.sdf" />
            <SpawnModel   name="spawn_model" model_name="green_cylinder" x="rand" y="rand" z="rand" R="rand" P="rand" Y="rand" file_path="<path_to_the_model_folder>/model.sdf" />
        </Sequence>
     </BehaviorTree>
    
 </root>
```
In the image below you can see the behavior tree defined in the xml file.

![spawn_xml.png](https://github.com/dobots/scenario_runner/blob/main/img/spawn_xml.png)

The behavior tree starts with the root, then in a sequence, it executes its child nodes. The first one is a repeat node, which will execute its child node 10 times. Its child node is a spawn node, which will spawn a blue cylinder. After spawning 10 blue cylinders at a random position and orientation, the next spawn node will be executed. It will spawn a red_cylinder. Then, the last node will be executed which will spawn a green cylinder.

**src folder:** Contains the source files, which will execute the behavior tree and all the necessary functions.
In the previous example, we executed the `spawn_multiple.cpp` file, which called the `spawn_multiple.xml` behaviour tree description file.

The `spawn_multiple.cpp` file looks like the following:
```
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "spawn_model.hpp"

using namespace BT;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
    factory.registerNodeType<SpawnModel>("SpawnModel");   
 
  //Create a tree from the provided xml file 
    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/scenarios/xml/spawn_multiple.xml");
         
   // To "execute" a Tree you need to "tick" it.
    tree.tickRoot();

  rclcpp::shutdown();
  return 0;
}
```
At first, we need to include the behaviortree library, the cpp library and our custom function to spawn models (*spawn_model.hpp*).

Then in the main function, we register our custom node type called "SpawnModel".

Then we read the xml file and create a tree from it. 

Finally, we "tick" the tree node-by-node until the entire sequence is executed.

**spawn_model.hpp:** This function will be discussed in more detail in the next section. If you would like to use the already available functions and create your own behavior trees, you don't need to worry about what is inside this function. On the other hand, if you would like to create a new custom function, it can be helpful to understand the logic behind it.

**CMakeLists.txt:** We need to modify our CMakeLists file and include our executables.
In addition, we need to add the behavior tree library to the list of dependencies:
```
find_package(behaviortree_cpp_v3 REQUIRED)
set(dependencies
  ..
  behaviortree_cpp_v3
)  
```

We also need to include our executable so ROS can find it:
```
add_executable(spawn_multiple src/spawn_multiple.cpp)
ament_target_dependencies(spawn_multiple rclcpp std_msgs gazebo_ros_pkgs gazebo_msgs behaviortree_cpp_v3)

install(TARGETS spawn_multiple DESTINATION lib/${PROJECT_NAME})
```

**package.xml:** We need to include the behavior tree dependency into our package.xml file as well:
```
  <depend>behaviortree_cpp_v3</depend>
```

## List of available functions and how to call them in the xml file

 ### SpawnModel:
 ```
 <SpawnModel   name="spawn_model" model_name="" x="" y="" z="" R="" P="" Y="" x_rmax="" y_rmax="" z_rmax="" R_rmax="" P_rmax="" Y_rmax="" file_path="<path_to_your_file>" />
 ```
 > If you would like to use a random position for the model use: x="rand", y="rand", etc. instead of a value x_rmax, y_rmax, etc. stands for the maximum range within the random number should be generated
 
 It is possible to spawn both urdf and sdf models:
 ```
  <SpawnModel   name="spawn_model" model_name="my_robot" file_path="<path_to_your_urdf_file>/urdf/my_robot.urdf" />
  ```
  
  ```      
<SpawnModel   name="spawn_model" model_name="cow" x="10" y="0" z="0,6"  R="0" P="0" Y="0"  file_path="<path_to_your_sdf_file>/cow_model/model.sdf" />
```
As already mentioned it is also possible to spawn models at a random position:
```
<SpawnModel   name="spawn_model" model_name="green_cylinder" x="rand" y="rand" z="rand" R="rand" P="rand" Y="rand" file_path="<path_to_your_sdf_file>/model.sdf" />
```
 
 ### RunROSNode
 It is possible to call any ROS node with this function. You just need to provide the package name, and the name of the executable. It is optional to provide any parameters to the ROSnode.
 
 ```
 <RunROSNode   name="run_node" package_name=""  executable_name=""  params="" />
 ```
 e.g.:
```
<RunROSNode   name="run_node" package_name="move_actors"  executable_name="pub_modelstate.py"  params="--node='pub_modelstate' --name 'cow' --x '10' --y '0' --z '0.6'" />
```

 ### CheckProximity
It is a condition node. It might be useful to use it with the "retry until succesful" decorator node.             
```
<RetryUntilSuccesful num_attempts="-1"> 
            		<CheckProximity name="check_proximity" ego_model_name="blue_cylinder" actor_model_name="red_cylinder" min_distance="20"/>
            </RetryUntilSuccesful>
```

## Running nodes in parallel
To be able to run behavior tree nodes in parallel, you need to edit your function from synchronous action node to asynchronous action node:
```
// AsyncActionNode (asynchronous action) with an input port.
class SpawnModel : public AsyncActionNode
{
  public:
    YourNodeName(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
       ...   
     }
```


## Spawn model function
As already mentioned in the previous section, if you would like to use the already available functions and create your own behavior trees, you don't need to worry about what is inside this function. On the other hand, if you would like to create a new custom function, it can be helpful to understand the logic behind it.

Let's examine this code piece-by-piece.

At first we need to include all the necessary libraries:
```
#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"
...
```
Notice, that we've included the gazebo service `spawn_entity` and the gazebo service `get_model_list`.

Then we create a synchronous action node, just like we have already seen in the behavior tree tutorials.

In the config, we intialise the ROS node and the necessary clients, which will call the `spawn_entity` and `get_model_list` services.

Then we need to provide the list of input ports. The ports used here should be the same as the ports used in the xml file, when the SpawnModel behaviour tree node is called.
```
// SyncActionNode (synchronous action) with an input port.
class SpawnModel : public SyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    SpawnModel(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
       node = rclcpp::Node::make_shared("spawn_entity_client");
       client_spawn = node->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
       client_model = node->create_client<gazebo_msgs::srv::GetModelList>("get_model_list");
       srand(time(0));
    
     }

    // It is mandatory to define this static method to list the input ports
    static PortsList providedPorts()
    {
        return { 
           InputPort<std::string>("model_name"),
           InputPort<std::string>("file_path"),
           InputPort<std::string>("x"),      InputPort<std::string>("y"),      InputPort<std::string>("z"), 
           InputPort<std::string>("R"),      InputPort<std::string>("P"),      InputPort<std::string>("Y"),
           InputPort<std::string>("x_rmax"), InputPort<std::string>("y_rmax"), InputPort<std::string>("z_rmax"),
           InputPort<std::string>("R_rmax"), InputPort<std::string>("P_rmax"), InputPort<std::string>("Y_rmax"),
           };
    }
```



In the next section, the "real" work happens, when we overwrite the virtual tick function. This is also based on the behavior trees tutorials.

```
 // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
      ...
        //Check whether the required inputs are provided. If not throw an error
        if (!getInput<std::string> ("model_name", model_name))
        {
            throw BT::RuntimeError("missing required input [model_name]");
        }
        
        if (!getInput<std::string> ("file_path", file_path))
        {
            throw BT::RuntimeError("missing required input [file_path]");
        }
```
First, we need to check whether the required ports are provided. If not we return an error. The rest of the ports are optional. If not provided, we will use a default value. 

```        
        // Waiting for spawn service /save
        while (!client_spawn->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service /spawn_entity.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for service /spawn_entity to appear...");
        }
        
        // Waiting for model list service /save
        ...
```
Then we will wait for the spawn service and the model_list_service to become available.     


```                   
        //Read file and store the received position
        std:: string xml_desc = readFile(node, file_path);
        ...          
```
In this section, we read the file path, open the xml description of the robot in this path and process it. Then we store the description in the xml_desc variable.  (for the full implementation, check the source file: https://github.com/dobots/scenario_runner/blob/main/scenarios/src/spawn_model.hpp) 


```         
        ...
        initial_pose.position.x = received_pose["x"];
        initial_pose.position.y = received_pose["y"];
        initial_pose.position.z = received_pose["z"];
        
        tf2::Quaternion q;
        q.setRPY(received_pose["R"],received_pose["P"],received_pose["Y"]);
        
        initial_pose.orientation.w = q[0];
        initial_pose.orientation.x = q[1];
        initial_pose.orientation.y = q[2];
        initial_pose.orientation.z = q[3];    
```
Then we read the received pose, in case it is "rand", we generate a random number. In case the "rmax" variable is provided as well, we update the maximum range within the random numbers are generated. Then we store the result in the initial_pose variable. (for the full implementation, check the source file: https://github.com/dobots/scenario_runner/blob/main/scenarios/src/spawn_model.hpp) 


```              
        
        //Send request to GetModelList service to update the model_name if it already exists
        auto request_model_list = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();
        auto result_model_list = client_model->async_send_request(request_model_list);
        
         // Wait for the result and update the name
         ...
```
Then we send a request to the `get_model_list` service to receive a list of models in the simulation. If the model name already exists we will append a number to its name.

```               
            
            
        //Send the request to the SpawnEntity service
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = model_name;
        request->xml = xml_desc;
        request->initial_pose = initial_pose;
        
        auto result = client_spawn->async_send_request(request);
```
After all this processing we are ready to call the spawn service, with all the required parameters.


```        
        if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Unable to call /spawn_entity");
            return BT::NodeStatus::FAILURE;
        }
        
         RCLCPP_INFO(node->get_logger(), "Waiting for result of spawning an entity...");
        //rclcpp::shutdown();
              
        // print some messages for feedback
        std::cout << "[ Spawn model: " << model_name << " spawned ]" <<std::endl;
        return NodeStatus::SUCCESS;
    }
    ...
```
If  it can successfully spawn the entity it will return success, otherwise it will return waiting or failure.

## Steps to create your own scenario
The necessary steps to create your own scenario will be familiar from the `bt_demo` tutorial. We need to follow the same steps as we did already in the first tutorial (https://github.com/dobots/scenario_runner/tree/main/bt_demo#3-create-the-source-files-to-check-battery-level) 

### 1. Create an xml description.
I would suggest copying the [spawn_multiple.xml](https://github.com/dobots/scenario_runner/blob/main/scenarios/xml/spawn_multiple.xml) file. Then try to modify it slightly and save it under a new name.

### 2. Create a .cpp file.
We need to create a .cpp file which will read the xml description, load the functions, and execute the behaviour tree. For this step, I would suggest to copy the [spawn_multiple.cpp](https://github.com/dobots/scenario_runner/blob/main/scenarios/src/spawn_multiple.cpp) file. Then modify the name of the xml file to load your own xml description. 
>**Note:** If you would like to use additional behaviour tree nodes in your xml file, you need to include their functions in your .cpp file, and register them in the main part of the .cpp file.


### 3. Modify CMakeLists.txt
The last step is to include our new .cpp file in the CMakeLists, so after building the package ROS2 can find our executable. 
The behaviour_tree library should be already listed in the `find_package` section.

```
#Add the executable and name it so you can run your node using ros2 run
add_executable(<your_file_name> src/<your_file_name>.cpp)
ament_target_dependencies(<your_file_name> rclcpp std_msgs behaviortree_cpp_v3)

#Finally, add the install(TARGETS..) section so ros2 run can find your executable
install(TARGETS <your_file_name> DESTINATION lib/${PROJECT_NAME})
```

### 4. Build your package

```
source /opt/ros/foxy/setup.bash
cd ~/<your_ws>
colcon build --packages-select scenarios
. install/setup.bash
```

### 5. Open a second terminal for Gazebo:

```
source /usr/share/gazebo/setup.sh
ros2 launch gazebo_ros gazebo.launch.py
```

### 6. In the first terminal, where we have built the package, let's run our new scenario:

```
ros2 run scenarios <your_file_name>
```



## Steps to create your custom node function for a scenario
In case you need functionality, which is not implemented yet, you might need to create your own functions. Therefore, in the next section, we will discuss the structure of 2 behaviour tree functions, which can call ROS2 services.

At first, we will have a look at a function called `autodockclient`. It can be find in the the ADlink-ROS/BT_ros2 repository: https://github.com/Adlink-ROS/BT_ros2/blob/master/src/autodock_client.hpp

The structure of these functions is very similar. 

**Orange colour:** At first, you need to select a name for your class and initialize it with the same name. 

**Green colour:** Second, you need to select from the BT library, which class would you like to inherit from. In the next examples, we will use a synchronous action node. 

**Red colour:** In the initialization part, you need to initialize the ROS services, and ROS nodes, you would like to use. 

**Blue colour:** Then you need to overwrite the `tick()` function. This is part is really, important! This is where you will include the functionality that you are expecting from this behaviour tree node. In addition, you need to return SUCCESS,RUNNING,or FAILURE NodeStates.

<p align="center">
<img src="https://github.com/dobots/scenario_runner/blob/main/img/autodockclient.png" width = "700" /> 
</p>
For the full implementation please visit: https://github.com/Adlink-ROS/BT_ros2/blob/master/src/autodock_client.hpp


In the image below, you can compare the structure with the previous function. The overall structure is the same. The interesting part is what happens in the `tick()` function, which can be seen in the full implementation.

<p align="center">
<img src="https://github.com/dobots/scenario_runner/blob/main/img/interruptevent.png" width = "700" /> 
</p>

For the full implementation please visit: https://github.com/Adlink-ROS/BT_ros2/blob/master/src/interrupt_event.hpp


## Check proximity function

The proximity check function checks, whether two models are within a given radius of each other.
It requires two model names and a distance value as an input. If the models are within the received distance it returns success, otherwise, it returns failure. 

For this node, we need Gazebo to publish the model states. To publish the model states we need to include the `gazebo_ros_state` plugin into our .sdf world file. 
```
<sdf version="1.6">
<world name="default">
...
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
<update_rate>1.0</update_rate>
</plugin>
...

</world>
</sdf>
```

There is an empty world file, including this plugin in our repository, which can be launched as the following:
```
cd ~/<your_ros2>_ws
source /opt/ros/foxy/setup.bash 
source /usr/share/gazebo/setup.sh 
. install/setup.bash

ros2 launch scenarios load_empty_world_gazebo.launch.py
```

Then in a new terminal we can launch our scenario containing a proximity check node:
```
cd ~/<your_ros2>_ws
source /opt/ros/foxy/setup.bash 
source /usr/share/gazebo/setup.sh 
. install/setup.bash

ros2 run scenarios spawn_prox
```

Then in the terminal, it prints out the position of the received models, and their distance. If this distance is smaller than the received distance it returns SUCCESS, otherwise, it returns FAILURE.


<p align="center">
<img src="https://github.com/dobots/scenario_runner/blob/main/img/prox_success.png" width = "700" /> 
</p>


