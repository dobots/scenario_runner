#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"

#include <math.h>  
#include <chrono>
#include <cstdlib>
#include <memory>


#include <fstream>
#include <iostream>
#include "../../pugixml-1.12/src/pugixml.cpp"

#include <map>
#include <tf2/LinearMath/Quaternion.h>
#include<ctime>

#include<string>

//For debugging
#include <typeinfo>

using namespace BT;
using namespace std;



// ConditionNode  with an input port.
class CheckProximity : public ConditionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    CheckProximity(const string& name, const NodeConfiguration& config)
      : ConditionNode(name, config)
    {
       node = rclcpp::Node::make_shared("check_proximity_client");
       //The get_entity_state service is provided by the libgazebo_ros_state.so plugin. You need to load it in your world file. It is not ported yet to ROS2 to be able to load it from a launch file.
       client_model = node->create_client<gazebo_msgs::srv::GetEntityState>("/get_entity_state");
    
     }

    // It is mandatory to define this static method to list the input ports
    static PortsList providedPorts()
    {
        return { 
           InputPort<string>("ego_model_name"),
           InputPort<string>("actor_model_name"),
           InputPort<string>("min_distance"),
           };
    }
 
 
     //Function to send request to GetEntityState and get the position of the given model
    geometry_msgs::msg::Point  requestModelState(string model_name){

    	auto request = make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = model_name;
        auto result = client_model->async_send_request(request);
        geometry_msgs::msg::Point model_pos;
        
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
             RCLCPP_INFO(node->get_logger(), "Result model state received");
             //get the state of models
             auto model_state = result.get()->state;
             model_pos = model_state.pose.position;
             
             //cout << "[ Position of: " << model_name << " x:"<<to_string(model_pos.x)<<" y:"<<to_string(model_pos.y)<< " z:"<<to_string(model_pos.z)<<endl;
             
             
         }else {
             RCLCPP_ERROR(node->get_logger(), "Failed to call service /get_entity_state");
         }
         
        return model_pos;
    }
    
            
     
    // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
        string ego_model_name;
        string actor_model_name;
        string min_distance;
       
        
        //Check whether the required inputs are provided. If not throw an error
        if (!getInput<string> ("ego_model_name", ego_model_name)){
            throw BT::RuntimeError("missing required input [ego_model_name]");
        }
        
        if (!getInput<string> ("actor_model_name", actor_model_name)){
            throw BT::RuntimeError("missing required input [actor_model_name]");
        }
             
             
        if (!getInput<string> ("min_distance", min_distance)){
            throw BT::RuntimeError("missing required input [min_distance]");
        }     
             
             
        //Send request to GetEntityState and get the position of the ego model       
        geometry_msgs::msg::Point model_pos= requestModelState(ego_model_name);
        cout << "Position of: " << ego_model_name << " x:"<<to_string(model_pos.x)<<" y:"<<to_string(model_pos.y)<< " z:"<<to_string(model_pos.z)<<endl;
        
         
        //Send request to GetEntityState and get the position of the actor model
        geometry_msgs::msg::Point actor_pos= requestModelState(actor_model_name);
        cout << "Position of: " << actor_model_name << " x:"<<to_string(actor_pos.x)<<" y:"<<to_string(actor_pos.y)<< " z:"<<to_string(actor_pos.z)<<endl;
        

	//Calculate distance between the two models
	float distance = sqrt( pow((model_pos.x - actor_pos.x),2) + pow((model_pos.y - actor_pos.y),2) + pow((model_pos.z - actor_pos.z),2));
	cout << "Distance: " << distance <<endl;


        // Return success or failure based on the distance	
	if (distance <= stof(min_distance)) {
	     RCLCPP_INFO(node->get_logger(), "CheckProximity:  NodeStatus::SUCCESS");
	     return NodeStatus::SUCCESS;	     
	     
	} else {
	     RCLCPP_INFO(node->get_logger(), "CheckProximity:  NodeStatus::FAILURE");
	     return NodeStatus::FAILURE;     
	}
        
       RCLCPP_INFO(node->get_logger(), "CheckProximity:  NodeStatus::RUNNING");
       return NodeStatus::RUNNING;
       
    }
    
    private:
        shared_ptr<rclcpp::Node> node;
        rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr client_model;

};

