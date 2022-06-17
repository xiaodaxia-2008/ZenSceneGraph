#include <SceneGraph/Node.h>
#include <iostream>
#include <spdlog/spdlog.h>

int main()
{
    using namespace sg;
    auto root = Node::Create("Root");
    auto node1 = Node::Create();
    root->AddChild(node1);
    auto node_robot = Node::Create("Robot");
    root->AddChild(node_robot);
    auto node_robot1 = Node::Create("Robot");
    auto node_tool = Node::Create("Tool");
    node_robot1->AddChild(node_tool);
    root->AddChild(node_robot1);
    auto node_robot2 = Node::Create("Robot");
    auto node_tool1 = Node::Create("Tool");
    node_robot2->AddChild(node_tool1);
    root->AddChild(node_robot2);

    fmt::print("Number of children: {}\n", root->GetNumChildren());
    std::string description;
    std::cout << root->PrintSceneGraph(description);
    description.clear();
    std::cout << node_robot2->PrintSceneGraph(description);

    return 0;
}