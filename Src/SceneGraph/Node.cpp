#include "Node.h"
#include <unordered_set>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace sg
{
static inline std::string IndentSpace(std::size_t indent)
{
    return std::string(indent, ' ');
}

NodeSPtr Node::Create(const std::string &name)
{
    return NodeSPtr(new Node(name));
}

Node::Node(const std::string &name)
{
    m_local_pose.setIdentity();
    SetName(name);
}

Node::~Node() { SPDLOG_TRACE("{} is destructed!", *this); }

void Node::SetName(const std::string &name)
{
    static std::unordered_map<std::string, std::size_t> node_names;
    if (auto &n = node_names[name]; n > 0) {
        m_name = name.empty() ? fmt::format("Node_{}", n)
                              : fmt::format("{}_{}", name, n);
        ++n;
    }
    else {
        m_name = name.empty() ? "Node" : name;
        node_names[m_name] = 1;
    }
}

const std::string &Node::GetName() const { return m_name; }

NodeSPtr Node::GetParent() const { return m_parent.lock(); }

const std::list<NodeSPtr> &Node::GetChildren() const { return m_children; }

std::size_t Node::GetNumChildren() const { return m_children.size(); }

NodeSPtr Node::GetChild(std::size_t idx) const
{
    if (idx >= GetNumChildren()) {
        SPDLOG_WARN("Index {} is larger than number of children {}", idx,
                    GetNumChildren());
        return nullptr;
    }
    auto it = m_children.begin();
    std::size_t i = 0;
    while (i++ < idx) {
        ++it;
    }
    return *it;
}

void Node::AddChild(NodeSPtr node)
{
    if (!node) {
        SPDLOG_WARN("node is nullptr!");
        return;
    }
    if (std::find(m_children.begin(), m_children.end(), node)
        == m_children.end()) {
        if (auto parent = node->GetParent()) {
            parent->RemoveChild(node);
        }
        m_children.push_back(node);
        node->SetParent(shared_from_this());
    }
    else {
        SPDLOG_DEBUG("{} is already a child of {}", *node, *this);
    }
}

void Node::SetParent(NodeSPtr node) { m_parent = node; }

void Node::RemoveChild(NodeSPtr node)
{
    if (!node) {
        SPDLOG_WARN("node is nullptr!");
        return;
    }
    auto it = std::find(m_children.begin(), m_children.end(), node);
    if (it != m_children.end()) {
        m_children.erase(it);
    }
    else {
        SPDLOG_WARN("{} is not a child of {}", *node, *this);
    }
}

void Node::RemoveChild(std::size_t idx) { RemoveChild(GetChild(idx)); }

void Node::SetLocalPose(const Pose &pose) { m_local_pose = pose; }

void Node::SetWorldPose(const Pose &pose)
{
    if (auto parent = GetParent()) {
        SetLocalPose(parent->GetWorldPose().inverse() * pose);
    }
    else {
        SetLocalPose(pose);
    }
}

const Pose &Node::GetLocalPose() const { return m_local_pose; }

Pose Node::GetWorldPose() const
{
    if (auto parent = GetParent()) {
        return parent->GetWorldPose() * GetLocalPose();
    }
    return GetLocalPose();
}

void Node::SetUserData(const std::string &key, UserDataSPtr data) const
{
    m_user_data_map[key] = data;
}

UserDataSPtr Node::GetUserData(const std::string &key) const
{
    return m_user_data_map[key];
}

std::string &Node::PrintSceneGraph(std::string &description,
                                   std::size_t indent) const
{
    description += IndentSpace(indent) + (GetNumChildren() > 0 ? "+" : "-");
    description += GetName();
    description += "\n";
    for (auto child : GetChildren()) {
        child->PrintSceneGraph(description, indent + 2);
    }
    return description;
}

std::ostream &operator<<(std::ostream &out, const Node &node)
{
    out << "<Node " << node.GetName() << " at " << &node << "\n>";
    return out;
}


void to_json(json &j, const NodeSPtr &node)
{
    j = json{
        {"name", node->GetName()},
        {"local_pose", node->GetLocalPose()},
        {"children", node->GetChildren()},
    };
}

void from_json(const json &j, NodeSPtr &node)
{
    node->SetName(j.at("name").get<std::string>());
    node->SetLocalPose(j.at("local_pose").get<Pose>());
    auto n = j.at("children").size();
    for (auto i = 0u; i < n; ++i) {
        auto child = Node::Create();
        j.at("children").at(i).get_to(child);
        node->AddChild(child);
    }
}
} // namespace sg

namespace Eigen
{
void to_json(json &j, const Matrix4d &pose)
{
    j = fmt::format(
        "{}", pose.format(Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ",")));
}

void from_json(const json &j, Matrix4d &pose)
{
    auto str = j.get<std::string>();
    std::vector<std::string> vec_str;
    boost::algorithm::trim_if(str, boost::is_any_of(","));
    boost::algorithm::split(vec_str, str, boost::is_any_of(","),
                            boost::algorithm::token_compress_on);
    std::vector<double> vec;
    for (auto &s : vec_str) {
        auto v = boost::lexical_cast<double>(s);
        SPDLOG_DEBUG("parse string [{}] to {}", s, v);
        vec.push_back(v);
    }
    pose = Eigen::Map<Matrix4d>(vec.data(), 4, 4);
}
} // namespace Eigen