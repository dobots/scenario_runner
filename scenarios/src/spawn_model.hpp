#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/get_model_list.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>


#include <fstream>
#include <iostream>
#include "../../pugixml-1.12/src/pugixml.cpp"

#include <map>
#include <tf2/LinearMath/Quaternion.h>
#include<ctime>

//For debugging
#include <typeinfo>

const int MAX_RANGE=25;

using namespace BT;
using namespace std;


//opens the file and the file_path and stores it content in the xml_desc variable
auto readFile(shared_ptr<rclcpp::Node> node, string file_path){
   string xml_desc;

   RCLCPP_INFO(node->get_logger(), "Loading entity XML from file: %s",file_path.c_str());
   
   //open the file
   ifstream ifile;
         ifile.open(file_path);
         if(!ifile){
            RCLCPP_INFO(node->get_logger(),"Error: specified file %s does not exist", file_path.c_str());
         }else{
            while ( !ifile.eof() ) { // keep reading until end-of-file
            	string line;
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
          RCLCPP_INFO(node->get_logger(),"Invalid XML: %s", string(res.description()).c_str());
          }
          
     return xml_desc;          
  }




// SyncActionNode (synchronous action) with an input port.
class SpawnModel : public SyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    SpawnModel(const string& name, const NodeConfiguration& config)
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
           InputPort<string>("model_name"),
           InputPort<string>("file_path"),
           InputPort<string>("x"),
           InputPort<string>("y"),
           InputPort<string>("z"), 
           InputPort<string>("R"),
           InputPort<string>("P"),
           InputPort<string>("Y"),
           InputPort<string>("x_rmax"),
           InputPort<string>("y_rmax"),
           InputPort<string>("z_rmax"),
           InputPort<string>("R_rmax"),
           InputPort<string>("P_rmax"),
           InputPort<string>("Y_rmax"),
           };
    }
 
    
    //Function to convert the received pose to initial pose. If the received pose is rand, generate a random position
    geometry_msgs::msg::Pose convertReceivedPose(){
             
        // Form requested Pose from arguments             
        geometry_msgs::msg::Pose initial_pose;
        //Optional x,y,z, R,P,Y
        map<string, float> received_pose;
        received_pose["x"] = 0;
        received_pose["y"] = 0;
        received_pose["z"] = 0;
        received_pose["R"] = 0;
        received_pose["P"] = 0;
        received_pose["Y"] = 0;
               
        
        string value;
        for(auto item: received_pose) {
            if (getInput<string> (item.first, value)){
                //if rand is received generate a random number
                if( value == "rand"){

                   string max_range_str;
                   int max_range;
                   if(getInput<string> ( (item.first + "_rmax"), max_range_str)){
                     //cout<<"max range found!!!"<<item.first<<endl;
                     max_range = stoi(max_range_str);
                   }else{
                   max_range= MAX_RANGE;
                   }
                    
                   received_pose.at(item.first) = rand() % (2*max_range+1) -max_range;
                   //if it is the z axis, then generate only positive numbers
                   if(item.first == "z"){ received_pose.at(item.first) = rand() % max_range;}
                 
                 
                //if a real value is received use it 
                }else{         	
            	received_pose.at(item.first) = stof(value);
            	}           	
            }
         }
         
        initial_pose.position.x = received_pose["x"];
        initial_pose.position.y = received_pose["y"];
        initial_pose.position.z = received_pose["z"];
        
        tf2::Quaternion q;
        q.setRPY(received_pose["R"],received_pose["P"],received_pose["Y"]);
        
        initial_pose.orientation.w = q[0];
        initial_pose.orientation.x = q[1];
        initial_pose.orientation.y = q[2];
        initial_pose.orientation.z = q[3];    
        
        return initial_pose;
        }
        
        //Function to append a number to the model name in case it already exists
        string updateModelName(string model_name){
        
        auto request_model_list = make_shared<gazebo_msgs::srv::GetModelList::Request>();
        auto result_model_list = client_model->async_send_request(request_model_list);
        
         // Wait for the result.
         if (rclcpp::spin_until_future_complete(node, result_model_list) == rclcpp::FutureReturnCode::SUCCESS){
             RCLCPP_INFO(node->get_logger(), "Result model list received");
             //get the list of models
             auto model_names = result_model_list.get()->model_names;
             
             int model_pcs=1;
             string orig_model_name = model_name;
             for(int i= 0; i<int(model_names.size()); i++){
                 //if the name already exists append a number to it and start the loop again with the new name
                 if(model_names[i] == model_name){
                     //cout<<model_names[i]<<" "<< model_name<<endl;
                     model_name = orig_model_name + to_string(model_pcs); 
                     model_pcs++;
                     i = 0;
                 }
             }
             
         } else {
             RCLCPP_ERROR(node->get_logger(), "Failed to call service /get_model_list");
         }
         
         return model_name;       
        }
     
     
     
     
     
    // Override the virtual function tick()
    virtual NodeStatus tick() override
    {
        string model_name;
        string file_path;
       
        
        //Check whether the required inputs are provided. If not throw an error
        if (!getInput<string> ("model_name", model_name)){
            throw BT::RuntimeError("missing required input [model_name]");
        }
        
        if (!getInput<string> ("file_path", file_path)) {
            throw BT::RuntimeError("missing required input [file_path]");
        }
        
                
        // Waiting for spawn service /save
        while (!client_spawn->wait_for_service(chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service /spawn_entity.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for service /spawn_entity to appear...");
        }
        
        // Waiting for model list service /save
        while (!client_model->wait_for_service(chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service /get_model_list.");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(node->get_logger(), "Waiting for service /get_model_list to appear...");
        }
        
        
        
        //Read file
         string xml_desc = readFile(node, file_path);
        
        //Calculate the initial pose based on the received values        
        geometry_msgs::msg::Pose initial_pose = convertReceivedPose();
             
                 
        //Send request to GetModelList service to update the model_name if it already exists
        model_name = updateModelName(model_name);
        
        
        //Send the request to the SpawnEntity service
        auto request = make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = model_name;
        request->xml = xml_desc;
        request->initial_pose = initial_pose;
        auto result = client_spawn->async_send_request(request);

        
        if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Unable to call /spawn_entity");
            RCLCPP_INFO(node->get_logger(), "SpawnModel: NodeStatus::FAILURE");
            return BT::NodeStatus::FAILURE;
        }
        
         RCLCPP_INFO(node->get_logger(), "Waiting for result of spawning an entity...");
              
        // print some messages for feedback
        cout << "[ Spawn model: " << model_name << " spawned ]" <<endl;
        RCLCPP_INFO(node->get_logger(), "SpawnModel: NodeStatus::SUCCESS");
        return NodeStatus::SUCCESS;
    }
    
    private:
        shared_ptr<rclcpp::Node> node;
        rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client_spawn;
        rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr client_model;

};

