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

#include <map>

//For debugging
#include <typeinfo>


//opens the file and the file_path and stores it content in the xml_desc variable
auto readFile(std::shared_ptr<rclcpp::Node> node, std::string file_path){
   std::string xml_desc;

   RCLCPP_INFO(node->get_logger(), "Loading entity XML from file: %s",file_path.c_str());
   
   //open the file
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
          
     return xml_desc;          
  }

       
          
          
          

/*auto quaternion_from_euler(float roll, pitch, yaw):
    		cy = math.cos(yaw * 0.5)
    		sy = math.sin(yaw * 0.5)
    		cp = math.cos(pitch * 0.5)
    		sp = math.sin(pitch * 0.5)
    		cr = math.cos(roll * 0.5)
    		sr = math.sin(roll * 0.5)

    		q = [0] * 4
    		q[0] = cy * cp * cr + sy * sp * sr
    		q[1] = cy * cp * sr - sy * sp * cr
    		q[2] = sy * cp * sr + cy * sp * cr
    		q[3] = sy * cp * cr - cy * sp * sr

    	return q*/


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
       client = node->create_client<gazebo_msgs::srv::SpawnEntity>("spawn_entity");
    
     }

    // It is mandatory to define this static method to list the input ports
    static PortsList providedPorts()
    {
        return { 
           InputPort<std::string>("model_name"),
           InputPort<std::string>("file_path"),
           InputPort<std::string>("x"),
           InputPort<std::string>("y"),
           InputPort<std::string>("z"), 
           InputPort<std::string>("R"),
           InputPort<std::string>("P"),
           InputPort<std::string>("Y"),
           };
    }

    // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
        std::string model_name;
        std::string pose_x;
        std::string pose_y;
        std::string pose_z;
        std::string pose_R;
        std::string pose_P;
        std::string pose_Y;
        std::string file_path;
       
        
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
        std:: string xml_desc = readFile(node, file_path);
                
        
        // Form requested Pose from arguments
        geometry_msgs::msg::Pose pose_received;       
        
        //Optional x,y,z, R,P,Y
        std::map<std::string, float> received_pose;
        received_pose["x"] = 0;
        received_pose["y"] = 0;
        received_pose["z"] = 0;
        received_pose["R"] = 0;
        received_pose["P"] = 0;
        received_pose["Y"] = 0;
        
        std::string value;
        for(auto item: received_pose) {
            if (getInput<std::string> (item.first, value)){           	
            	received_pose.at(item.first) = std::stof(value);           	
            }
         }
         
        pose_received.position.x = received_pose["x"];
        pose_received.position.y = received_pose["y"];
        pose_received.position.z = received_pose["z"];
        
        
        pose_received.orientation.w = 0;
        pose_received.orientation.x = 0;
        pose_received.orientation.y = 0;
        pose_received.orientation.z = 0;     
        
        //python
        /*initial_pose = Pose()

        q = quaternion_from_euler(self.args.R, self.args.P, self.args.Y)
        initial_pose.orientation.w = q[0]
        initial_pose.orientation.x = q[1]
        initial_pose.orientation.y = q[2]
        initial_pose.orientation.z = q[3]  */  
        
        
    
            
            
        //Send the request to the SpawnEntity service
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = model_name;
        request->xml = xml_desc;
        request->initial_pose = pose_received;
        
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

