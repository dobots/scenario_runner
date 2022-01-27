//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include "drone_mission_functions.hpp"

using namespace BT;


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
 
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

 
    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    //factory.registerSimpleCondition("DeleteModel", std::bind(DeleteModel));
    //factory.registerNodeType<DeleteModel>("DeleteModel");
 
    
    // SimpleActionNodes can not define their own method providedPorts().
    // We should pass a PortsList explicitly if we want the Action to 
    // be able to use getInput() or setOutput();
    PortsList model_name_ports = { InputPort<std::string>("model_name") };
    factory.registerSimpleAction("DeleteModel", DeleteModel,             model_name_ports ); 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/scenarios/xml/ch_class1.xml");
    
    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();
    //SleepMS(150);
  
  rclcpp::shutdown();
  return 0;
}
