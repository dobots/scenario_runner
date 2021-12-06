//Include the behaviortree
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"


#include "rclcpp/rclcpp.hpp"


using namespace BT;

// We want to be able to use this custom type
struct Position2D { double x,y; };

// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
namespace BT
{
template <> inline Position2D convertFromString(StringView str)
{
    printf("Converting string: \"%s\"\n", str.data() );

    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        Position2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        return output;
    }
}
} // end namespace BT

class CalculateGoal: public SyncActionNode
{
public:
    CalculateGoal(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name,config)
    {}

    static PortsList providedPorts()
    {
        return { OutputPort<Position2D>("goal") };
    }

    NodeStatus tick() override
    {
        Position2D mygoal = {1.1, 2.3};
        setOutput<Position2D>("goal", mygoal);
        return NodeStatus::SUCCESS;
    }
};


class PrintTarget: public SyncActionNode
{
public:
    PrintTarget(const std::string& name, const NodeConfiguration& config):
        SyncActionNode(name,config)
    {}

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char*  description = "Simply print the goal on console...";
        return { InputPort<Position2D>("target", description) };
    }

    NodeStatus tick() override
    {
        auto res = getInput<Position2D>("target");
        if( !res )
        {
            throw RuntimeError("error reading port [target]:", res.error());
        }
        Position2D target = res.value();
        printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y );
        return NodeStatus::SUCCESS;
    }
};

/** The tree is a Sequence of 4 actions
*  1) Store a value of Position2D in the entry "GoalPosition"
*     using the action CalculateGoal.
*
*  2) Call PrintTarget. The input "target" will be read from the Blackboard
*     entry "GoalPosition".
*
*  3) Use the built-in action SetBlackboard to write the key "OtherGoal".
*     A conversion from string to Position2D will be done under the hood.
*
*  4) Call PrintTarget. The input "goal" will be read from the Blackboard
*     entry "OtherGoal".
*/


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
 
  // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");

    auto tree = factory.createTreeFromFile("/home/reka/foxy2_ws/src/bt_demo/xml/t03.xml");
    
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
