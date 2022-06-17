#pragma once

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif
#define EIGEN_DEFAULT_IO_FORMAT                                                \
    Eigen::IOFormat(4, 0, ", ", ",\n", "[", "]", "[", "]")
#include <Eigen/Dense>

#include <nlohmann/json.hpp>

#include <map>
#include <memory>
#include <string>
#include <list>
#include <ostream>

using json = nlohmann::ordered_json;

namespace sg
{
using Pose = Eigen::Matrix4d;

class UserData : public std::enable_shared_from_this<UserData>
{
public:
    virtual ~UserData() {}

protected:
    UserData() = default;
};

using UserDataSPtr = std::shared_ptr<UserData>;

class Node;
using NodeSPtr = std::shared_ptr<Node>;
using NodeWPtr = std::weak_ptr<Node>;

class Node : public std::enable_shared_from_this<Node>
{
    friend void to_json(json &j, const Node &node);
    friend void from_json(const json &j, Node &node);

public:
    static NodeSPtr Create(const std::string &name = "");

    virtual ~Node();

    void SetName(const std::string &name);
    const std::string &GetName() const;

    NodeSPtr GetParent() const;
    const std::list<NodeSPtr> &GetChildren() const;
    std::size_t GetNumChildren() const;
    NodeSPtr GetChild(std::size_t idx) const;
    void AddChild(NodeSPtr node);
    void RemoveChild(NodeSPtr node);
    void RemoveChild(std::size_t idx);

    void SetLocalPose(const Pose &pose);
    void SetWorldPose(const Pose &pose);
    const Pose &GetLocalPose() const;
    Pose GetWorldPose() const;

    void SetUserData(const std::string &key, UserDataSPtr data) const;
    UserDataSPtr GetUserData(const std::string &key) const;

    std::string &PrintSceneGraph(std::string &description,
                                 std::size_t indent = 0u) const;

protected:
    Node(const std::string &name);
    void SetParent(NodeSPtr node);

    std::string m_name;
    Pose m_local_pose;
    NodeWPtr m_parent;
    std::list<NodeSPtr> m_children;

    mutable std::unordered_map<std::string, UserDataSPtr> m_user_data_map;
};

void to_json(json &j, const NodeSPtr &node);
void from_json(const json &j, NodeSPtr &node);

std::ostream &operator<<(std::ostream &out, const Node &node);
} // namespace sg
namespace Eigen
{
void to_json(json &j, const Matrix4d &pose);
void from_json(const json &j, Matrix4d &pose);
}