// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once
#include <RVS/Planner/Core/Graph/Types.h>
#include <RVS/Planner/Core/Graph/TerminateCondition.h>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/graphviz.hpp>
#include <optional>

namespace RVS
{

///@addtogroup planner
///@{
namespace graph
{

/**
 * @brief Graph is designed to provide basic functions needed to build and search (Dijkstra, Astar ...)
 * on a abstract graph  
 * 
 * @tparam VertexProperties: the properties type by which a vertex represents
 * @tparam EdgeProperties: the information contains by an edge 
 */
template <typename VertexProperties = NoProperty,
          typename EdgeProperties = NoProperty>
class Graph : public std::enable_shared_from_this<
                  Graph<VertexProperties, EdgeProperties>>
{
public:
    /**... If the VertexList template parameter of the adjacency_list was vecS,
     * then all vertex descriptors, edge descriptors, and iterators for the
     * graph are invalidated by this operation. The builtin vertex_index_t
     * property for each vertex is renumbered so that after the operation the
     * vertex indices still form a contiguous range [0, num_vertices(g)). ...
     * refer to
     * https://www.boost.org/doc/libs/1_47_0/libs/graph/doc/adjacency_list.html
     * */


    // boost type alias
    using BoostGraph =
        typename boost::adjacency_list<boost::vecS, boost::vecS,
                                       boost::directedS, VertexProperties,
                                       EdgeProperties>;
    using BVertexIterator =
        typename boost::graph_traits<BoostGraph>::vertex_iterator;
    using BOutEdgeIterator =
        typename boost::graph_traits<BoostGraph>::out_edge_iterator;
    using BInEdgeIterator =
        typename boost::graph_traits<BoostGraph>::in_edge_iterator;
    using PredesessorMap = std::vector<VertexDescriptor>;
    using DistanceMap = std::vector<double>;
    using HeuristicFunction =
        std::function<double(VertexProperties, VertexProperties)>;

    /** @brief heurictic defined for A star search */
    class AStarHeuristic : public boost::astar_heuristic<BoostGraph, double>
    {
    public:
        AStarHeuristic(VertexDescriptor goal,
                       const HeuristicFunction &heuristic_function,
                       Graph *graph)
            : m_heuristic_function(heuristic_function), m_graph(graph)
        {
            m_goal_properties = graph->GetVertexProperties(goal);
        }
        double operator()(VertexDescriptor u)
        {
            VertexProperties v_p = m_graph->GetVertexProperties(u);
            return m_heuristic_function(v_p, m_goal_properties);
        }

    private:
        VertexProperties m_goal_properties;
        HeuristicFunction m_heuristic_function;
        Graph *m_graph;
    };
    
    /** @brief default constructor */ 
    Graph() {}

    /** @brief copy constructor **/ 
    Graph(const Graph &others) { m_bgraph = others.m_bgraph; }

    /** @brief default destructor **/ 
    virtual ~Graph() = default;


    /** @brief add vertex by a VertexProperties
     *  @param vertex_properties: properties of the vertex
    */
    VertexDescriptor AddVertex(const VertexProperties &vertex_properties)
    {
        VertexDescriptor vd = boost::add_vertex(m_bgraph);
        m_bgraph[vd] = vertex_properties;
        return vd;
    }

    /**
     * @brief add edge between two vertices
     * @param v1: the descriptor of the first vertex
     * @param v2: the descriptor of the second vertex
     * @param ep: properties of the edge
     * @param is_directed:
     * add (v1 -> v2) if `is_directed`
     * add (v1 -> v2) and (v2 -> v1) if not `is_directed`
     */
    bool AddEdge(const VertexDescriptor &v1, const VertexDescriptor &v2,
                 const EdgeProperties &ep, bool is_directed = true)
    {
        bool inserted = false;
        typename boost::graph_traits<BoostGraph>::edge_descriptor ed;
        std::tie(ed, inserted) =
            boost::add_edge(boost::vertex(v1, m_bgraph),
                            boost::vertex(v2, m_bgraph), ep, m_bgraph);

        if (!inserted) {
            RVS_ERROR("insert edge ({}, {}) failed", v1, v2);
            return false;
        }
        RVS_DEBUG("insert edge ({}, {})", v1, v2);
        if (!is_directed) return AddEdge(v2, v1, ep, true);

        return true;
    }



    /**
     * @brief remove a vertex by its descriptor
     * @param vertex: descriptor of the vertex
     */
    void RemoveVertex(VertexDescriptor vertex)
    {
        boost::remove_vertex(vertex, m_bgraph);
        return;
    }

    /**
     * @brief remove vertices
     * @param vertices: descriptors of the vertex
     */
    void RemoveVertices(const std::vector<VertexDescriptor> &vertices)
    {
        std::vector<VertexDescriptor>::const_iterator v_i;
        for (v_i = vertices.begin(); v_i != vertices.end(); ++v_i) {
            boost::remove_vertex(*v_i, m_bgraph);
        }
        return;
    }

    /** @brief get the number of vertices */
    size_t GetNumVertices() const { return boost::num_vertices(m_bgraph); }

    /** @brief get the number of edges */
    size_t GetNumEdges() const { return boost::num_edges(m_bgraph); }


    /** @brief get the properties of a vertex 
     *  @param vertex: descriptor of the vertex
    */
    VertexProperties GetVertexProperties(VertexDescriptor vertex) const { return m_bgraph[vertex]; }

    /** @brief get all the `in vertices` of a vertex */
    std::vector<VertexDescriptor> GetInVertices(const VertexDescriptor &v) const 
    {
        std::vector<VertexDescriptor> in_vertices;
        BInEdgeIterator e_i, e_end;
        boost::tie(e_i, e_end) =
            boost::in_edges(boost::vertex(v, m_bgraph), m_bgraph);

        for (; e_i != e_end; ++e_i) {
            in_vertices.push_back(boost::source(*e_i, m_bgraph));
        }
        return in_vertices;
    }

    /** @brief get all the `out vertices` of a vertex */
    std::vector<VertexDescriptor> GetOutVertices(const VertexDescriptor &v) const 
    {

        std::vector<VertexDescriptor> out_vertices;
        BOutEdgeIterator e_i, e_end;
        boost::tie(e_i, e_end) =
            boost::out_edges(boost::vertex(v, m_bgraph), m_bgraph);
        for (; e_i != e_end; ++e_i) {
            out_vertices.push_back(boost::target(*e_i, m_bgraph));
        }

        return out_vertices;
    }

    /** @brief Dijkstra shortest path search from start to goal
     *  @param start: vertex descriptor of start vertex
     *  @param goal: vertex descriptor of goal vertex
     *  @param[out] path: a vector contains vertice of the shortest path 
     *  @param[out] path_cost: path cost
     *  @param term_cond: terminate condition, the process will be stoped
     *  if term_cond is satisfied
     *  @return true for success, else false
     */
    bool DijkstraPath(VertexDescriptor start, VertexDescriptor goal,
                      std::vector<VertexDescriptor> &path, double &path_cost,
                      TerminateConditionPtr term_cond =
                          std::shared_ptr<NonTerminateCondition>())
    {
        DistanceMap distance_map;
        PredesessorMap predesessor_map;

        distance_map.resize(GetNumVertices());
        predesessor_map.resize(GetNumVertices());

        try {
            boost::dijkstra_shortest_paths(
                m_bgraph, start,
                boost::weight_map(boost::get(&EdgeProperties::weight, m_bgraph))
                    .distance_map(&distance_map[0])
                    .predecessor_map(&predesessor_map[0])
                    .visitor(DijstraVisitor(term_cond, {goal})));
        }
        catch (ExitFlag<VertexDescriptor> e) {
            // we terminated before examining the goal, it is a failure
            RVS_DEBUG("terminated before examining the goal");
            if(e.exit_type == ExitType_TerminateCondition){
                return false;
            }
        }

        if (!GetOptimalPath(start, goal, predesessor_map, path)) {
            RVS_DEBUG("cannot find optimal path from vertex {} to {}", start,
                      goal);
            return false;
        }
        path_cost = distance_map[goal];

        return true;
    }

    /** @brief Dijkstra shortest path search from start to a goal set
     *  @param start: vertex descriptor of start vertex
     *  @param goal_set: vertex descriptor of goal vertex set
     *  @param[out] path: a vector contains vertice of the shortest path 
     *  @param[out] path_cost: path cost
     *  @param term_cond: terminate condition, the process will be stoped
     *  if term_cond is satisfied
     *  @return true for success, else false
     */
    bool DijkstraPath(VertexDescriptor start,
                      const std::vector<VertexDescriptor> &goal_set,
                      std::vector<VertexDescriptor> &path, double &path_cost,
                      TerminateConditionPtr term_cond =
                          std::shared_ptr<NonTerminateCondition>())
    {
        DistanceMap distance_map;
        PredesessorMap predesessor_map;

        distance_map.resize(GetNumVertices());
        predesessor_map.resize(GetNumVertices());

        try {
            boost::dijkstra_shortest_paths(
                m_bgraph, start,
                boost::weight_map(boost::get(&EdgeProperties::weight, m_bgraph))
                    .distance_map(&distance_map[0])
                    .predecessor_map(&predesessor_map[0])
                    .visitor(DijstraVisitor(term_cond)));
        }
        catch (ExitFlag<VertexDescriptor> e) {
            // do nothing
        }


        double min_path_cost = std::numeric_limits<double>::infinity();
        VertexDescriptor min_path_cost_goal = 0;
        for (auto const &goal : goal_set) {
            double cost = distance_map[goal];
            RVS_TRACE("goal : {}, cost: {}", goal, cost);
            if (cost < min_path_cost) {
                min_path_cost = cost;
                min_path_cost_goal = goal;
            }
        }

        if (min_path_cost_goal == 0) {
            RVS_ERROR("Unknown search error");
            return false;
        }

        path_cost = min_path_cost;
        bool res =
            GetOptimalPath(start, min_path_cost_goal, predesessor_map, path);

        if (!res) {
            RVS_DEBUG("cannot find optimal path from vertex {} to {}", start,
                      min_path_cost_goal);
            return false;
        }
        return true;
    }

    /** @brief Dijkstra shortest path search from start set to goal set
     *  @param start_set: vertex descriptor of start vertex set
     *  @param goal_set: vertex descriptor of goal vertex set
     *  @param[out] path: a vector contains vertice of the shortest path 
     *  @param[out] path_cost: path cost
     *  @param term_cond: terminate condition, the process will be stoped
     *  if term_cond is satisfied
     *  @return true for success, else false
     */
    bool DijkstraPath(const std::vector<VertexDescriptor> &start_set,
                      const std::vector<VertexDescriptor> &goal_set,
                      std::vector<VertexDescriptor> &path, double &path_cost,
                      TerminateConditionPtr term_cond =
                          std::shared_ptr<NonTerminateCondition>())
    {
        path_cost = std::numeric_limits<double>::infinity();
        bool any_success = false;
        for (const auto &start : start_set) {
            double curr_path_cost;
            std::vector<VertexDescriptor> curr_path;
            bool success = DijkstraPath(start, goal_set, curr_path,
                                        curr_path_cost, term_cond);
            any_success = (any_success) || success;
            if (!success) {
                continue;
            }
            if (curr_path_cost < path_cost) {
                path_cost = curr_path_cost;
                path = curr_path;
            }
        }
        return any_success;
    }

    /** @brief Astar shortest path search from start to goal
     *  @param start: vertex descriptor of start vertex
     *  @param goal: vertex descriptor of goal vertex
     *  @param[out] path: a vector contains vertice of the shortest path 
     *  @param[out] path_cost: path cost
     *  @param term_cond: terminate condition, the process will be stoped
     *  if term_cond is satisfied
     *  @return true for success, else false
     */
    bool AStarPath(VertexDescriptor start, VertexDescriptor goal,
                   const HeuristicFunction &heuristic_function,
                   std::vector<VertexDescriptor> &path, double &path_cost,
                   TerminateConditionPtr term_cond =
                       std::shared_ptr<NonTerminateCondition>())
    {

        typename Graph::DistanceMap distance_map;
        typename Graph::PredesessorMap predesessor_map;

        distance_map.resize(GetNumVertices());
        predesessor_map.resize(GetNumVertices());

        try {
            boost::astar_search(
                m_bgraph, start,
                Graph::AStarHeuristic(goal, heuristic_function, this),
                boost::weight_map(boost::get(&EdgeProperties::weight, m_bgraph))
                    .distance_map(&distance_map[0])
                    .predecessor_map(&predesessor_map[0])
                    .visitor(AStarVisitor(term_cond)));
        }
        catch (ExitFlag<VertexDescriptor> e) {
            VertexDescriptor exit_vertex = e.t;
            RVS_DEBUG("graph search exit after examining the vertex {}",
                      exit_vertex);
        }
        bool res = GetOptimalPath(start, goal, predesessor_map, path);
        path_cost = distance_map[goal];
        return res;
    }

    /**
     * @brief save graph rviz
     * @param file_name: path for saving graphviz
     */
    template<typename VertexPropertyWrite, typename EdgePropertyWrite>
    void SaveGraphviz(const std::string &file_name, const VertexPropertyWrite& vertex_property_write, 
                    const EdgePropertyWrite& edge_property_write)
    {
        std::filesystem::path file_path = file_name;
        if (file_path.extension() != ".dot") {
            file_path += ".dot";
        }

        std::ofstream dot_file(file_path.string());
        boost::dynamic_properties dp;
        dp.property("node_id", boost::get(vertex_property_write, m_bgraph));
        dp.property("label", boost::get(edge_property_write, m_bgraph));
        boost::write_graphviz_dp(dot_file, m_bgraph, dp);
        RVS_INFO("Save Graphviz to file: {}", file_path.string());
    }

    /**
     * @brief save graph rviz
     * @param file_name: path for saving graphviz
     */
    template<typename EdgePropertyWrite>
    void SaveGraphviz(const std::string &file_name, 
                    const EdgePropertyWrite& edge_property_write)
    {
        std::filesystem::path file_path = file_name;
        if (file_path.extension() != ".dot") {
            file_path += ".dot";
        }

        std::ofstream dot_file(file_path.string());
        boost::dynamic_properties dp;
        dp.property("node_id", boost::get(boost::vertex_index, m_bgraph));
        dp.property("label", boost::get(edge_property_write, m_bgraph));
        boost::write_graphviz_dp(dot_file, m_bgraph, dp);
    }

    /**
     * @brief clear all the vertex and edge information
     */
    void ClearGraph() { m_bgraph.clear(); }

protected:
    /**
     * @brief Get the Optimal Path object
     * 
     * @param s: start vertex
     * @param g: goal vertex
     * @param pred_map: predesessor map from any path finding algorithm
     * @param[out] path: a vector contains vertice of the shortest path 
     * @param reverse: return path in reverse order if true
     * @return true for success, else false
     */
    bool GetOptimalPath(const VertexDescriptor &s, const VertexDescriptor &g,
                        const PredesessorMap &pred_map,
                        std::vector<VertexDescriptor> &path,
                        bool reverse = false)
    {

        RVS_DEBUG("start vertex: {}", s);
        RVS_DEBUG("goal vertex: {}", g);
        std::vector<VertexDescriptor> reverse_path;
        VertexDescriptor vertex = g;
        while (vertex != s) {
            RVS_DEBUG("current vertex: {}", vertex);
            reverse_path.push_back(vertex);
            VertexDescriptor pred = pred_map[vertex];
            if (pred == vertex) ///< predessessor no found
                return false;
            vertex = pred;
            if (vertex == boost::graph_traits<BoostGraph>::null_vertex()) {
                RVS_ERROR("Cannot find vaild parent for vertex: {}", vertex);
                path.clear();
                return false;
            }
        }

        reverse_path.push_back(s);

        if (!reverse) {
            std::reverse(reverse_path.begin(), reverse_path.end());
        }

        path = reverse_path;

        return true;
    }

    BoostGraph m_bgraph;
};


/**
 * @brief WeightedGraph is a graph whose edge properties is EdgeWeight
 * (it's a wrapper struct for `double`). Which means each edge has only single
 * property: a weight (or cost).
 * 
 * @tparam VertexProperties: the properties type by which a vertex represents
 */
template <typename VertexProperties>
class WeightedGraph : public Graph<VertexProperties, EdgeWeight>
{
public:
    WeightedGraph()
    : Graph<VertexProperties, EdgeWeight>()
    {
        
    } 
    
    ~WeightedGraph() {}

    /**
     * @brief add edge between two vertices
     * @param v1: the descriptor of the first vertex
     * @param v2: the descriptor of the second vertex
     * @param ep: properties of the edge
     * @param is_directed:
     * add (v1 -> v2) if `is_directed`
     * add (v1 -> v2) and (v2 -> v1) if not `is_directed`
     */
    bool AddEdge(const VertexDescriptor &v1, const VertexDescriptor &v2,
                 const double weight, bool is_directed = true)
    {
        EdgeWeight ep(weight);
        return Graph<VertexProperties, EdgeWeight>::AddEdge(v1, v2, ep, is_directed);
    }

};

///@}
} // namespace graph
} // namespace RVS