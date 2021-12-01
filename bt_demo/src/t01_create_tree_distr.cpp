//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"


#include "rclcpp/rclcpp.hpp"

#include "simple_functions.hpp"

using namespace BT;

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
 
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

   // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // you may also use C++11 lambdas instead of std::bind
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", 
                                 std::bind(&GripperInterface::open, &gripper));
    factory.registerSimpleAction("CloseGripper", 
                                 std::bind(&GripperInterface::close, &gripper));

        
    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 

    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/bt_demo/xml/t01.xml");
    
    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    printTreeRecursively(tree.rootNode());

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();
  
  rclcpp::shutdown();
  return 0;
}
