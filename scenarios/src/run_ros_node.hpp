#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>

using namespace BT;
using namespace std;


// SyncActionNode (synchronous action) with an input port.
class RunROSNode : public AsyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    RunROSNode(const string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    	//ROS nodes subscribers, publishers, server, clients
    	//node = rclcpp::Node::make_shared("spawn_entity_client");
    }
    


    // It is mandatory to define this static method to list the input ports
    static PortsList providedPorts()
    {
        return { 
           InputPort<string>("package_name"),
           InputPort<string>("executable_name"),
           InputPort<string>("params")
           };
    }
    
     
    // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
        string package_name;
        string executable_name;
        string params;
       
        
        //Check whether the required inputs are provided. If not throw an error
        //If some of the inputs are optional, remove this check
        if (!getInput<string> ("package_name", package_name)){
            throw BT::RuntimeError("missing required input [package_name]");
        }
        
        if (!getInput<string> ("executable_name", executable_name)) {
            throw BT::RuntimeError("missing required input [executable_name]");
        }
        
        getInput<string>("params", params);

        // Send shell commands
        //system("source /home/reka/foxy2_ws/install/setup.bash"); 
        //system("source /opt/ros/foxy/setup.bash");  
        system(("ros2 run " + package_name + " " + executable_name + " " + params).c_str()); 
        
  
           
        // print some messages for feedback
        //cout << "Package name: "<<package_name << "  node_name: "<< node_name<<endl;
        return NodeStatus::SUCCESS;
    }
    
    private:
        //shared_ptr<rclcpp::Node> node;
        //rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client_spawn;
        //rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr client_model;*/
    
};

