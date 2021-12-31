// Copyright (c) RVBUST, Inc - All rights reserved
#pragma once

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <list>
#include <vector>
#include <limits>

#include <RVS/Common/Macros.h>
#include <RVS/Common/Logger.h>
#include <RVS/Planner/Core/Types.h>
#include <RVS/Kinematics/Utils.h>

namespace RVS
{
/// @addgroup Planner
/// @{

enum GraphSearchMethod
{
    GraphSearchMethod_Undef = 0,
    GraphSearchMethod_AStar = 1, ///< A* algorithm
    GraphSearchMethod_Dijkstra = 2 ///< Dijkstra's algorithm
};

enum HeuristicMethod
{
    HeuristicMethod_Undef = 0,
    HeuristicMethod_Distance = 1, ///< the distance to the goal
    HeuristicMethod_Duration = 2 ///< the length of time to the goal
};

enum RotationAxis
{
    RotationAxis_Undef = 0, ///< undefined rotated axis
    RotationAxis_X = 1, /// rotate around x-axis
    RotationAxis_Y = 2, ///< rotate around y-axis
    RotationAxis_Z = 3 ///< rotate around z-axis
};

class GraphSearchConfig
{
public:
    typedef typename boost::listS listS;
    typedef typename boost::vecS vecS;
    typedef typename boost::undirectedS undirectedS;
    typedef typename boost::no_property no_property;
    typedef typename boost::edge_weight_t edge_weight_t;
    typedef typename boost::vertex_index_t vertex_index_t;
    using MetricSpace = std::function<double(
        const JointVector &, const JointVector &, const CVecXd &)>;

    /**
     * @brief Construct a new graph search config object
     *
     * @param method_name The name of method searching graph
     * @param gsm The method that is adopted to search minimized cost
     * path. At present, both A* and Dijkstra methods are provided
     *
     */
    GraphSearchConfig(const std::string &method_name,
                      GraphSearchMethod gsm = GraphSearchMethod_Undef);
    // destruction function
    virtual ~GraphSearchConfig(){};

    /**
     * @brief Set the graph nodes
     *
     * @param nodes The node of each level will be multiple joint configurations
     * of manipualtor corresponding to a single target pose if nodes are
     * descirbed in joint space. Otherwise, it is essential to resample the
     * poses of nodes in order to yield the required graph nodes.
     *
     */
    void SetGraphNodes(const std::vector<ViapointPtr> &nodes)
    {
        m_nodes = nodes;
    }

    /**
     * @brief To check whether it has found a feasible minimized cost path by
     * given method
     *
     * @return true The result if a feasible path has found
     * @return false The result if failing to do it
     *
     */
    bool HasFoundGoal();

    /**
     * @brief Get the graph search name
     *
     * @return std::strings The name of current graph search
     *
     */
    const std::string &GetGraphSearchName() const { return m_method_name; }

    /**
     * @brief Get the shortest (minimized cost) path
     *
     * @return const std::vector<JointVector>& The sequential joint
     * configurations along given target poses
     *
     */
    const std::vector<JointVector> &GetShortestPath() const
    {
        return m_shortest_path_nodes;
    }

    /**
     * @brief Set the joint weights which will be used to measure the cost
     * between two nodes that are resided in adjacent level
     *
     * @param new_weights The weight value of each component of joint
     * configuration
     *
     */
    void SetJointWeights(const CVecXd &new_weights);

    /**
     * @brief Get the joint weights
     *
     * @return const CVecXd& The current weight value of each component of joint
     * configuration
     */
    const CVecXd &GetJointWeights() const { return m_joint_weights; }

    /**
     * @brief Set the sample parameters which is aimed at generating required
     * nodes when input graph nodes are targets pose instead of joint
     * configurations
     *
     * @param manip The manipulator that will move following given target poses
     * @param rot_axis The reference rotation axis in order to generate new
     * target poses, i.e., it can take X,Y or Z axis
     * @param sample_int The sampling interval in order to generate new target
     * pose around given reference rotation axis, the range of value is (0,2*pi]
     *
     */
    void SetSampleParams(std::shared_ptr<Manipulator> manip,
                         RotationAxis rot_axis, double sample_int);

    /**
     * @brief Set the reference start target which is the joint configuration
     * that manipulator arrives at it first.
     *
     * @param init_joint_config The input initial joint configuration
     *
     */
    void SetReferStartTarget(const JointVector &init_joint_config);

    /**
     * @brief Get the reference start target
     *
     * @return const JointVector&  The current reference start target
     *
     */
    const JointVector &GetReferStartTarget() const
    {
        return *m_start_joint_config;
    }

    /**
     * @brief Set the metric func that measures the cost between nodes from
     * adjacent level
     *
     * @param dist_func The "distance" function, i.e., it may be arbitrarily
     * legal callable expression
     *
     */
    void SetMetricFunc(const MetricSpace &dist_func)
    {
        m_dist_func = dist_func;
    }

    /**
     * @brief To keep the idential joint configuraiton at the start and end of
     * motion of manipulator, which exclusively adjusts to the result of
     * resampling the nodes which represent target poses
     *
     * @param is_same If it is true, the joint configuration will be equal.
     * Otherwise, it may be different from each other
     *
     */
    void EnableSameAtStartAndEnd(bool is_same)
    {
        m_is_same_start_and_end = is_same;
    }

    /**
     * @brief To signify whether the graph search has been well configured
     *
     * @return RVSReturn The result (Succeeded or Uninitialized)
     *
     */
    RVSReturn IsConfigured() { return m_is_configured; }

protected:
    // initialize the related parameters with some graph search method
    virtual bool _InitializeParameters() = 0;

    // find minimal cost path in a given graph
    virtual void _CalcShortestPath() = 0;

    // Generate the initial targets with joint configurations and return their
    // indices
    virtual std::vector<std::vector<size_t>>
    _GenerateInitTargets(std::vector<JointVector> &raw_targets);

    // Resample the targets at task space in order to get required nodes in
    // joint space subsequently
    virtual void
    _ResampleTargetsAtTS(std::vector<JointVector> &init_targets,
                         std::vector<std::vector<size_t>> &region_idx);

    std::string m_method_name; ///< the name of graph search
    GraphSearchMethod
        m_search_method_type; ///< shortest path search method type
    std::vector<ViapointPtr> m_nodes; ///< nodes that make up (compose) a graph
    std::vector<JointVector>
        m_shortest_path_nodes; ///< found shortest path nodes
    size_t m_dof; ///< the active freedom of degree of robot arm
    CVecXd m_joint_weights;
    std::vector<double> m_dof_vel_limits; ///< velocity limits of each joint
    MetricSpace m_dist_func; ///< callable object in order to calculate
                             ///< "distance" between two targets
    std::shared_ptr<Manipulator> m_manip;
    RotationAxis m_rot_axis; ///< referrence axis when resampling the targets at
                             ///< task space
    double m_sample_reso; ///< the resolution of resampling targets
    std::shared_ptr<JointVector>
        m_start_joint_config; ///< it holds the start joint configuration which
                              ///< is given by user
    bool
        m_is_same_start_and_end; ///< check whether it ought to keep
                                 ///< identical joint configuration when
                                 ///< arriving at the same target (wrt position)
    RVSReturn m_is_configured; ///< the indicator that signifies whether the
                               ///< graph search has been well configured
    bool m_found_goal; ///< the indicator that a valid path has been found by
                       ///< graph search
};

/**
 * @brief the sign that a shortest path has been found by graph searching
 * algorithm
 *
 */
template <class Vertex>
struct FoundGoal
{
    FoundGoal(const Vertex &v_idx) : m_vertex_idx(v_idx) {}
    Vertex m_vertex_idx;
};

/**
 * @brief The euclidean distance heuristic (the difference of joint angles) is
 * targeted at guiding the graph search algorithm to find a minimized cost path
 *
 */
template <class Graph, class CostType, class JointAngles, class DistWeights>
class DistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor AVertex;
    DistanceHeuristic(const JointAngles &joint_angles,
                      const std::vector<AVertex> &goals,
                      const DistWeights &weights)
    {
        m_joint_angles = joint_angles;
        m_goals = goals;
        m_weights = weights;
    }
    // destruction function
    virtual ~DistanceHeuristic(){};
    CostType operator()(AVertex curr_vertex)
    {
        CostType angle_euclidean_dist =
            std::numeric_limits<CostType>::infinity();
        for (const auto &v : m_goals) {
            CostType temp =
                NearZero(m_weights.norm())
                    ? (m_joint_angles[v] - m_joint_angles[curr_vertex])
                          .WeightedNorm()
                    : (m_joint_angles[v] - m_joint_angles[curr_vertex])
                          .WeightedNorm(m_weights);
            if (temp < angle_euclidean_dist) {
                angle_euclidean_dist = temp;
            }
        }
        return angle_euclidean_dist;
    }

private:
    JointAngles m_joint_angles;
    std::vector<AVertex> m_goals;
    DistWeights m_weights;
};


/**
 * @brief The duration heuristic (duration of moving joints) is targeted at
 * guiding the graph search algorithm to find a minimized cost path
 *
 */
template <class Graph, class CostType, class JointAngles, class DofVelLimits>
class DurationHeuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor AVertex;
    DurationHeuristic(const JointAngles &joint_angles,
                      const std::vector<AVertex> &goals,
                      const DofVelLimits &dof_vel_limits)
    {
        m_joint_angles = joint_angles;
        m_goals = goals;
        m_dof_vel_limits = dof_vel_limits;
    }
    // destruction function
    virtual ~DurationHeuristic(){};
    CostType operator()(AVertex curr_vertex)
    {
        CostType duration_c = std::numeric_limits<CostType>::infinity();
        for (const auto &v : m_goals) {
            CostType temp;
            auto angle_diff =
                (m_joint_angles[v] - m_joint_angles[curr_vertex]).Coeffs();
            CVecXd temp_dof_limits = Eigen::Map<CVecXd, Eigen::Unaligned>(
                m_dof_vel_limits.data(), m_dof_vel_limits.size());
            angle_diff = angle_diff.array() / temp_dof_limits.array();
            // take its absolute value to keep maximum value positive
            angle_diff = angle_diff.cwiseAbs();
            std::vector<double> temp_res(angle_diff.data(),
                                         angle_diff.data() + angle_diff.size());
            auto max_time_iter =
                std::max_element(temp_res.begin(), temp_res.end());
            temp = *max_time_iter;
            if (temp < duration_c) {
                duration_c = temp;
            }
        }
        return duration_c;
    }

private:
    JointAngles m_joint_angles;
    DofVelLimits m_dof_vel_limits;
    std::vector<AVertex> m_goals;
};

/**
 * @brief The visitor that stops querying when the given goal is found (A*
 * algorithm)
 *
 */
template <class Vertex>
class AStarGoalVisitor : public boost::default_astar_visitor
{
public:
    AStarGoalVisitor(const std::vector<Vertex> &goals) : m_goals(goals) {}
    // default destruction function
    virtual ~AStarGoalVisitor() {}
    template <class Graph>
    void examine_vertex(Vertex curr_vertex, Graph &graph)
    {
        RVS_UNUSED(graph);
        auto res_it = std::find(m_goals.begin(), m_goals.end(), curr_vertex);
        if (res_it != m_goals.end()) {
            throw FoundGoal<Vertex>(*res_it);
        }
    }

private:
    std::vector<Vertex> m_goals;
};

class GraphSearchWithAStar : public GraphSearchConfig
{
public:
    // specify some types (refer to
    // https://www.boost.org/doc/libs/1_41_0/libs/graph/example/astar-cities.cpp)
    typedef
        typename boost::adjacency_list<listS, vecS, undirectedS, no_property,
                                       boost::property<edge_weight_t, double>>
            AJointConfigGraph;
    typedef typename boost::property_map<AJointConfigGraph, edge_weight_t>::type
        AWeightMap;
    typedef AJointConfigGraph::vertex_descriptor AVertex;
    typedef AJointConfigGraph::vertex_iterator AVertexIterator;
    typedef AJointConfigGraph::edge_descriptor AEdgeDescriptor;
    typedef AJointConfigGraph::edge_iterator AEdgeIterator;
    typedef std::pair<size_t, size_t> AEdge;
    typedef DistanceHeuristic<AJointConfigGraph, double,
                              std::vector<JointVector>, CVecXd>
        ADistanceHeuristic;
    typedef DurationHeuristic<AJointConfigGraph, double,
                              std::vector<JointVector>, std::vector<double>>
        ADurationHeuristic;
    typedef AStarGoalVisitor<AVertex> AStarGoalVisitorImp;
    using AVertexList = std::list<AVertex>;
    using AVertexVec = std::vector<AVertex>;
    /**
     * @brief Construct a new graph search with A* method
     *
     * @param dof  The degree of freedom of given manipulator
     *
     */
    GraphSearchWithAStar(size_t dof = 6);

    /**
     * @brief Construct a new graph search with A* method
     *
     * @param nodes The raw nodes of graph, i.e., it is required to be joint
     * configurations or target poses at the task space of manipulator
     * @param dof The degree of freedom given manipulator
     *
     */
    GraphSearchWithAStar(std::vector<ViapointPtr> nodes, size_t dof = 6);
    /**
     * @brief To update the joint limits with corresponding manipulator
     *
     * @param joint_limits The input joint limits whose dimension should be
     * consistent with degree of freedom of given manipulator
     *
     */
    void UpdateDofLimits(const std::vector<JointLimit> &joint_limits);

    /**
     * @brief The heuristic function that ushers the graph search to find the
     * minimized cost paths
     *
     * @param hm The heuristic function type. Two heuristic methods are
     * implemented at present
     */
    void UpdateHeuristic(HeuristicMethod hm) { m_heur_meth = hm; }

protected:
    // initialize the related parameters with A* algorithm
    virtual bool _InitializeParameters();

    // find minimal cost path in a given graph
    virtual void _CalcShortestPath();

private:
    AJointConfigGraph m_graph; ///< the graph described by boost library
    std::vector<JointVector>
        m_targets_at_cs; ///< all targets at configuration space
    std::vector<std::vector<size_t>>
        m_region_idx; ///<  joint configuration indices corresponding to the
                      ///<  targets
    HeuristicMethod
        m_heur_meth; ///< heuristic method which was applied in astar_search
};

/**
 * @brief The visitor that stops querying when the given goal is found
 * (Dijkstra's algorithm)
 *
 */
template <class Vertex>
class DijkstraGoalVisitor : public boost::default_dijkstra_visitor
{
public:
    DijkstraGoalVisitor(const std::vector<Vertex> &goals) : m_goals(goals) {}
    // destruction function
    virtual ~DijkstraGoalVisitor(){};
    template <class Graph>
    void examine_vertex(Vertex curr_vertex, Graph &graph)
    {
        RVS_UNUSED(graph);
        auto res_it = std::find(m_goals.begin(), m_goals.end(), curr_vertex);
        if (res_it != m_goals.end()) {
            throw FoundGoal<Vertex>(*res_it);
        }
    }

private:
    std::vector<Vertex> m_goals;
};

class GraphSearchWithDijkstra : public GraphSearchConfig
{
public:
    typedef boost::adjacency_list<listS, vecS, undirectedS, no_property,
                                  boost::property<edge_weight_t, double>>
        DJointConfigGraph;
    typedef boost::property_map<DJointConfigGraph, edge_weight_t>::type
        DWeightMap;
    typedef boost::property_map<DJointConfigGraph, vertex_index_t>::type
        DIndexMap;
    typedef boost::graph_traits<DJointConfigGraph>::vertex_descriptor DVertex;
    typedef boost::graph_traits<DJointConfigGraph>::vertex_iterator
        DVertexIterator;
    typedef boost::graph_traits<DJointConfigGraph>::edge_descriptor
        DEdgeDescriptor;
    typedef boost::graph_traits<DJointConfigGraph>::edge_iterator DEdgeIterator;
    typedef std::pair<size_t, size_t> DEdge;
    typedef DijkstraGoalVisitor<DVertex> DijkstraGoalVisitorImp;
    using DVertexList = std::list<DVertex>;
    using DVertexVec = std::vector<DVertex>;
    /**
     * @brief Construct a new graph search with Dijkstra's method
     *
     * @param dof  The degree of freedom of given manipulator
     *
     */
    GraphSearchWithDijkstra(size_t dof = 6);
    /**
     * @brief Construct a new graph search with Dijkstra's method
     *
     * @param nodes The raw nodes of graph, i.e., it is required to be joint
     * configurations or target poses at the task space of manipulator
     * @param dof The degree of freedom of given manipulator
     *
     */
    GraphSearchWithDijkstra(std::vector<ViapointPtr> nodes, size_t dof = 6);

protected:
    // initialize the related parameters with Dijkstra's algorithm
    virtual bool _InitializeParameters();

    // find minimal cost path in a given graph
    virtual void _CalcShortestPath();

private:
    DJointConfigGraph m_graph; ///< the graph described by boost library
    std::vector<JointVector>
        m_targets_at_cs; // all targets at configuration space
    std::vector<std::vector<size_t>>
        m_region_idx; ///< joint configuration indices at each goal
};

/// @}
} // namespace RVS