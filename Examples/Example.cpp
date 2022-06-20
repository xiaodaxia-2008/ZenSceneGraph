#include <SceneGraph/Node.h>
#include <iostream>
#include <spdlog/spdlog.h>

class MyUserData : public sg::UserData
{
public:
    MyUserData() {}
    std::string some_data = "some data";
};

int main()
{
    using namespace sg;
    Pose pose;
    pose.setIdentity();
    pose(0, 3) = 1.0;
    pose(1, 3) = 0.534;
    pose(2, 3) = 0.3;

    auto root = Node::Create("Root");
    auto node1 = Node::Create();
    root->AddChild(node1);
    auto node_robot = Node::Create("Robot");
    node_robot->SetWorldPose(pose);
    root->AddChild(node_robot);
    auto node_robot1 = Node::Create("Robot");
    auto node_tool = Node::Create("Tool");
    node_robot1->AddChild(node_tool);
    root->AddChild(node_robot1);
    auto node_robot2 = Node::Create("Robot");
    node_robot2->SetWorldPose(pose);
    auto node_tool1 = Node::Create("Tool");
    node_robot2->AddChild(node_tool1);
    node_tool1->SetLocalPose(pose);
    node_tool1->SetWorldPose(Pose::Identity());
    root->AddChild(node_robot2);

    fmt::print("Number of children: {}\n", root->GetNumChildren());
    std::string description;
    std::cout << root->PrintSceneGraph(description);
    description.clear();
    std::cout << node_robot2->PrintSceneGraph(description);
    std::cout << node_tool1->GetLocalPose() << std::endl;
    std::cout << node_tool1->GetWorldPose() << std::endl;

    UserDataSPtr my_udata(new MyUserData);
    node_robot1->SetUserData("MyUserData", my_udata);
    assert(!node_robot1->GetUserData("NonexistUserData"));
    std::cout << std::dynamic_pointer_cast<MyUserData>(
                     node_robot1->GetUserData("MyUserData"))
                     ->some_data
              << std::endl;

    json j = *root;
    std::cout << j << std::endl;
    NodeSPtr root2 = Node::Create();
    j.get_to(*root2);
    description.clear();
    std::cout << root2->PrintSceneGraph(description);
    return 0;
}