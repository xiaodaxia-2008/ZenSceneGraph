#include "Node.h"
#include <unordered_set>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

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
} // namespace sg