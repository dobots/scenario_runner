//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

//#include "drone_mission_functions.hpp"
#include "spawn_model.hpp"

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

using namespace BT;

/*void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}*/

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
 
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
    
    // SimpleActionNodes can not define their own method providedPorts().
    // We should pass a PortsList explicitly if we want the Action to 
    // be able to use getInput() or setOutput();
    //PortsList model_name_ports = { InputPort<std::string>("model_name"),  InputPort<std::string>("xml")};
    //factory.registerSimpleAction("SpawnModel", SpawnModel,             model_name_ports ); 
    
    factory.registerNodeType<SpawnModel>("SpawnModel");   
 
    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    //factory.registerSimpleCondition("SpawnModel", std::bind(SpawnModel));

        
    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/scenarios/xml/spawn_rand_max.xml");
    
    // This logger publish status changes using ZeroMQ. Used by Groot
    //PublisherZMQ publisher_zmq(tree);
    
    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();
    //SleepMS(150);
  
  rclcpp::shutdown();
  return 0;
}
