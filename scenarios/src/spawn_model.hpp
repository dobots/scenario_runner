#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace BT;

// SyncActionNode (synchronous action) with an input port.
class SpawnModel : public SyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    SpawnModel(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
       node = rclcpp::Node::make_shared("spawn_entity_client");
       client =
    node->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
    
     }

    // It is mandatory to define this static method to list the input ports
    static PortsList providedPorts()
    {
        return { InputPort<std::string>("model_name"), InputPort<std::string>("xml_desc") };
    }

    // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
        std::string model_name;
        std::string xml_desc;
        
        //Check whether the required inputs are provided. If not throw an error
        if (!getInput<std::string> ("model_name", model_name))
        {
            throw BT::RuntimeError("missing required input [model_name]");
        }
        
        if (!getInput<std::string> ("xml_desc", xml_desc))
        {
            throw BT::RuntimeError("missing required input [xml_desc]");
        }
        
        
        // Waiting for service /save
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service /spawn_entity.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for service /spawn_entity to appear...");
        }
        
        //Hardcoded xml description:
        xml_desc="<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></visual></link></model></sdf>";
        
        //Send the request to the SpawnEntity service
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = model_name;
        request->xml = xml_desc;
        
        auto result = client->async_send_request(request);

        // print some messages for feedback
        std::cout << "[ Spawn model: " << model_name << " spawned ]" <<std::endl;
        return NodeStatus::SUCCESS;
    }
    
    private:
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client;

};

