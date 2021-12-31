// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <iostream>

#include "Types.h"

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(GraphConstructorBase);


class EnvGraph : public std::enable_shared_from_this<EnvGraph>
{
public:
    const std::vector<JointVector> &GetNodes() const { return m_nodes; }

    size_t GetNodesNum() const { return m_nodes.size(); }

    void AddNode(const JointVector &node) { m_nodes.emplace_back(node); }

    void AddNodes(const std::vector<JointVector> &nodes)
    {
        for (size_t i = 0; i < nodes.size(); i++) {
            m_nodes.emplace_back(nodes[i]);
        }
    }

    void ClearGraph() { m_nodes.clear(); }

private:
    std::vector<JointVector> m_nodes;
};


/**
 * @brief The base class of graph constructor
 * When given a planning environment, it can generate the SE3 poses of the
 * specified characteristics(such as in container, Fan-shaped area surrounding
 * the robot or on the surface of the bonding box of the object, etc) and their
 * corresponding joint points, for the construction of the planning road map
 *
 * @todo The currently returned data structure type is a vector container.
 * Later, we will consider using the boost graph library to manage the data
 * structure.
 */
class GraphConstructorBase
    : public std::enable_shared_from_this<GraphConstructorBase>
{
public:
    explicit GraphConstructorBase(const std::string &name)
        : m_name(std::move(name))
    {
        RVS_TRACE("Constructing GraphConstructorBase");
        m_graph = std::make_shared<EnvGraph>();
    }

    virtual ~GraphConstructorBase()
    {
        RVS_TRACE("Destroying GraphConstructorBase");
    }

    struct Configuration : public ConfigurationBase
    {
    public:
        RVS_DECLARE_PTR_MEMBER(Configuration);

        /**
         * @brief Construct a new Configuration from Environment and
         * Manipulators
         *
         * @param env_in
         * @param manipulators_in
         */
        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in);

        virtual ~Configuration() = default;

        virtual bool Setup() override { return ConfigurationBase::Setup(); }

        virtual bool IsStateValid(const JointVector &state) const;
    };

    const std::string &GetName() const { return m_name; }

    virtual RVSReturn IsConfigured() const = 0;

    const Configuration::Ptr &GetConfiguration() const
    {
        return m_configuration;
    }

    virtual bool ConstructGraph() = 0;

    const std::vector<JointVector> &GetGraphNodes() const;

protected:
    std::string m_name;
    Configuration::Ptr m_configuration{nullptr};
    std::shared_ptr<EnvGraph> m_graph;
};
}; // namespace RVS