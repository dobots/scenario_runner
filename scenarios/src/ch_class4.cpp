//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include "drone_mission_functions.hpp"

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

 
    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("Wind", std::bind(Wind));

        
    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/scenarios/xml/ch_class4.xml");
    
    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();
    //SleepMS(150);
  
  rclcpp::shutdown();
  return 0;
}
