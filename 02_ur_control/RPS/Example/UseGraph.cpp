/**
 * @example UseGraph.cpp
 * @brief A lite use example for Graph
 *
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/Planner/Core/Graph/Graph.h>
#include <RVS/Planner/Core/Graph/TerminateCondition.h>
#include <RVS/Common/Eigen.h>

using namespace RVS;
using namespace RVS::graph;
void TestWeightedGraph()
{
    // create a vertex property that contains only the name of the vertex
    struct VertexName
    {
        char name;
    };

    // create a weighted graph (with edge cost type 'double') with `VertexName`
    typedef WeightedGraph<VertexName> MyGraph;

    // contruction graph
    MyGraph graph;

    // add vertices to the graph
    // it returns a VertexDescriptor as an unique ID of the vertex 
    VertexDescriptor A = graph.AddVertex(VertexName({'A'}));
    VertexDescriptor B = graph.AddVertex(VertexName({'B'}));
    VertexDescriptor C = graph.AddVertex(VertexName({'C'}));
    VertexDescriptor D = graph.AddVertex(VertexName({'D'}));
    VertexDescriptor E = graph.AddVertex(VertexName({'E'}));

    // connect vertices by weighted edge 
    graph.AddEdge(A, B, 1);
    graph.AddEdge(A, C, 2);
    graph.AddEdge(A, D, 3);
    graph.AddEdge(B, E, 1);
    graph.AddEdge(C, E, 1);
    graph.AddEdge(D, E, 1);

    // if terminate condition is triggered, path finding should be exit immediately
    auto term_cond = std::make_shared<NonTerminateCondition>();
    // if success, the vector should contains all the `VertexDescriptor` all along the path
    std::vector<VertexDescriptor> path;
    // path cost
    double path_cost;

    bool success = graph.DijkstraPath(A, E, path, path_cost, term_cond);
    if (!success) {
        RVS_ERROR("Dijkstra path search failed");
        return;
    }

    // save graphviz, with specified property
    // here, the vertex name and the edge cost will be written to the graph as label
    graph.SaveGraphviz("test_weighted_graph",  &VertexName::name, &EdgeWeight::weight);

    std::cout << "shortest path from A to E is: \n";
    std::vector<VertexDescriptor>::iterator v_i;
    for (v_i = path.begin(); v_i != path.end(); v_i++) {
        std::cout << "->" << graph.GetVertexProperties(*v_i).name;
    }
    std::cout << std::endl;
    std::cout << "with path cost: " << path_cost << std::endl;

    return;
}


typedef CVec2d Position;
typedef WeightedGraph<Position> Graph2D;

// helper function for evaluating edge cost
void AddEdgeToGraph(const VertexDescriptor &v1, const VertexDescriptor &v2,
                    Graph2D &graph)
{
    Position p1 = graph.GetVertexProperties(v1);
    Position p2 = graph.GetVertexProperties(v2);
    double edge_cost = (p1 - p2).norm();
    graph.AddEdge(v1, v2, edge_cost);
}

void TestAStarSearch()
{

    Graph2D graph;

    // add vertices to the graph
    // it returns a VertexDescriptor as an unique ID of the vertex
    VertexDescriptor v1 = graph.AddVertex(CVec2d(0, 0));
    VertexDescriptor v2 = graph.AddVertex(CVec2d(1, 0));
    VertexDescriptor v3 = graph.AddVertex(CVec2d(1, 1));
    VertexDescriptor v4 = graph.AddVertex(CVec2d(1, 2));
    VertexDescriptor v5 = graph.AddVertex(CVec2d(3, 2));

    // connect vertices by weighted edge 
    AddEdgeToGraph(v1, v2, graph);
    AddEdgeToGraph(v1, v3, graph);
    AddEdgeToGraph(v2, v3, graph);
    AddEdgeToGraph(v3, v4, graph);
    AddEdgeToGraph(v2, v5, graph);

    // for A star search, a heuristic function should be given
    // here we simply calculate the normed distance
    Graph2D::HeuristicFunction distance_heuristic = [](const Position &p1,
                                                       const Position &p2) {
        double distance = (p2 - p1).norm();
        return distance;
    };

    auto term_cond = std::make_shared<NonTerminateCondition>();

    std::vector<VertexDescriptor> path;
    double path_cost;
    bool success =
        graph.AStarPath(v1, v5, distance_heuristic, path, path_cost, term_cond);

    if (!success) {
        RVS_ERROR("AStar path search failed");
        return;
    }

    std::cout << "shorest path from v1 to v5 is\n";
    std::vector<VertexDescriptor>::iterator v_i;
    for (v_i = path.begin(); v_i != path.end(); v_i++) {
        Position position = graph.GetVertexProperties(*v_i);
        std::cout << "->" << "(" << position[0] << "," << position[1] << ")";
    }
    std::cout << std::endl;
    std::cout << "with path cost: " << path_cost << std::endl;

    return;
}

int main()
{

    TestWeightedGraph();
    TestAStarSearch();
    return 0;
}