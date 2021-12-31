// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once
#include <RVS/Common/Types.h>
#include <RVS/Planner/Core/Graph/TerminateCondition.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>


namespace RVS
{

namespace graph
{

/** @brief use `size_t` to index an unique vertex */
typedef size_t VertexDescriptor;
/** @brief use `size_t` to index an unique edge */
typedef size_t EdgeDescriptor;

/** @brief vertex index counts from 0 */
static constexpr VertexDescriptor gk_start_vertex = 0;


/**
 * @brief different types of exit when calling a search algorithm
 */
enum ExitType
{
    ExitType_Unknown,
    ExitType_TerminateCondition,
    ExitType_EarlyExit
};

template <typename T>
struct ExitFlag
{
    ExitFlag<T>(const T &t_in, const ExitType exit_type_in = ExitType_Unknown)
        : t(t_in), exit_type(exit_type_in)
    {
    }

    T t;
    ExitType exit_type;
};


/**
 * @brief default no property for graph template
 */
struct NoProperty
{
    // empty property struct
};

/**
 * @brief default property for weighted graph
 */
struct EdgeWeight
{
    // EdgeWeight(double);
    explicit EdgeWeight(double weight_ = 0.) : weight(weight_) {}

    double weight;
};


/**
 * @brief Dijkstra Vistor handles problem of early exit and termination
 */
class DijstraVisitor : public boost::dijkstra_visitor<>
{
public:
    /**
     * @brief Construct a new Dijstra Visitor object
     * 
     * @param term_cond: terminate condition
     * @param early_exit_vertices: searching will exit early if any vertex of `early_exit_vertices`
     *  is examined.
     */
    DijstraVisitor(
        TerminateConditionPtr &term_cond,
        const std::vector<VertexDescriptor> &early_exit_vertices = {})
        : m_term_cond(term_cond), m_early_exit_vertices(early_exit_vertices)
    {
    }

    template <typename Vertex, typename Graph>
    void examine_vertex(Vertex v, Graph & /*g*/) 
    {
        for (VertexDescriptor early_exit_vertex : m_early_exit_vertices) {
            if (v == early_exit_vertex) {
                RVS_DEBUG("graph search exit at examining vertex :{}", v);
                throw ExitFlag(v, ExitType_EarlyExit);
            }
        }
        if (m_term_cond->Eval()) {
            RVS_DEBUG("graph search exit at examining vertex :{}", v);
            throw ExitFlag(v, ExitType_TerminateCondition);
        }
    }

private:
    TerminateConditionPtr m_term_cond;
    std::vector<VertexDescriptor> m_early_exit_vertices;
};

/**
 * @brief Dijkstra Vistor handles problem of early exit and termination
 */
class AStarVisitor : public boost::astar_visitor<>
{
public:

    /**
     * @brief Construct a new Dijstra Visitor object
     * 
     * @param term_cond: terminate condition
     * @param early_exit_vertices: searching will exit early if any vertex of `early_exit_vertices`
     *  is examined.
     */
    AStarVisitor(TerminateConditionPtr term_cond,
                 const std::vector<VertexDescriptor> &early_exit_vertices = {})
        : m_term_cond(term_cond), m_early_exit_vertices(early_exit_vertices)
    {
    }

    template <typename Vertex, typename Graph>
    void examine_vertex(Vertex v, Graph & /*g*/)
    {
        for (VertexDescriptor early_exit_vertex : m_early_exit_vertices) {
            if (v == early_exit_vertex) {
                RVS_DEBUG("graph search exit at examining vertex :{}", v);
                throw ExitFlag(v, ExitType_EarlyExit);
            }
        }
        if (m_term_cond->Eval()) {
            RVS_DEBUG("graph search exit at examining vertex :{}", v);
            throw ExitFlag(v, ExitType_TerminateCondition);
        }
    }

private:
    TerminateConditionPtr m_term_cond;
    std::vector<VertexDescriptor> m_early_exit_vertices;
};

} // namespace graph
} // namespace RVS