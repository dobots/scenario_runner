#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>


#include <fstream>
#include <iostream>
#include "../../pugixml-1.12/src/pugixml.cpp"

//For debugging
#include <typeinfo>


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
        return { InputPort<std::string>("model_name"), InputPort<std::string>("file_path") };
    }

    // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
        std::string model_name;
        std::string file_path;
        std::string xml_desc;
        
        //Check whether the required inputs are provided. If not throw an error
        if (!getInput<std::string> ("model_name", model_name))
        {
            throw BT::RuntimeError("missing required input [model_name]");
        }
        
        if (!getInput<std::string> ("file_path", file_path))
        {
            throw BT::RuntimeError("missing required input [file_path]");
        }
        
        
        // Waiting for service /save
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service /spawn_entity.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for service /spawn_entity to appear...");
        }
        
        //Read file
        RCLCPP_INFO(node->get_logger(), file_path);
        RCLCPP_INFO(node->get_logger(), "Loading entity XML from file: %s",file_path.c_str());
        
         
         std::ifstream ifile;
         ifile.open(file_path);
         if(!ifile){
            RCLCPP_INFO(node->get_logger(),"Error: specified file %s does not exist", file_path.c_str());
         }else{
            while ( !ifile.eof() ) { // keep reading until end-of-file
            	std::string line;
            	ifile >> line;
            	xml_desc += line;
            	xml_desc += " ";	
            }
            ifile.close();
         }     
           
         //Parse xml to detect invalid xml before sending to gazebo
         pugi::xml_document doc;
         pugi::xml_parse_result res = doc.load_string(xml_desc.c_str());
         if(!res)
         {
          RCLCPP_INFO(node->get_logger(),"Invalid XML: %s", std::string(res.description()).c_str());
          }   
            
            
            
            
        //Send the request to the SpawnEntity service
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = model_name;
        request->xml = xml_desc;
        
        auto result = client->async_send_request(request);

        
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
    
    private:
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client;

};

