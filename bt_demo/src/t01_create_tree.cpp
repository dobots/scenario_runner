//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"


#include "rclcpp/rclcpp.hpp"


using namespace BT;

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Example of custom SyncActionNode (synchronous action)
// without ports.
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface
{
public:
    GripperInterface(): _open(true) {}

    NodeStatus open() {
        _open = true;
        std::cout << "GripperInterface::open" << std::endl;
        return NodeStatus::SUCCESS;
    }

    NodeStatus close() {
        std::cout << "GripperInterface::close" << std::endl;
        _open = false;
        return NodeStatus::SUCCESS;
    }

private:
    bool _open; // shared information
};




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
