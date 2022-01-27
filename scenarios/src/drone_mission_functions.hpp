#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


// Simple function that return a NodeStatus
BT::NodeStatus SpawnModel()
{
    std::cout << "[ SpawnModel: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}


// Simple function that return a NodeStatus
BT::NodeStatus Arm()
{
    std::cout << "[ Arm: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TakeOff()
{
    std::cout << "[ Take off: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

/*BT::NodeStatus FlyToB()
{
    std::cout << "[ Fly to B: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}*/


// SyncActionNode (synchronous action) with an input port.
class FlyToB : public SyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    FlyToB(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    { }

    // It is mandatory to define this static method.
    static PortsList providedPorts()
    {
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return { InputPort<std::string>("message") };
    }

    // As usual, you must override the virtual function tick()
    NodeStatus tick() override;
};   
    
BT::NodeStatus FlyToB::tick()
{
    std::cout << "[ Flying: STARTED ]" << std::endl;


    int count = 0;
    // Pretend that "computing" takes 2500 milliseconds.
    while (count++ < 250)
    {
        SleepMS(10);
    }

    std::cout << "[ Flying: FINISHED ]" << std::endl;
    return NodeStatus::SUCCESS;
}

    
  





BT::NodeStatus Land()
{
    std::cout << "[ Land: OK ]" << std::endl;
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

