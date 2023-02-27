#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <set>
#include <stack>
#include <algorithm>
#include <string>
#include <cstring>

class Graph {
public:
    Graph() = default;
    Graph(int V);
    ~Graph() = default;

    friend std::ostream& operator<< (std::ostream& os, const Graph& obj);

    int sendToPython();

    int getNumOfVertex();

    int getWeightOfEdge(int v1, int v2);

    // Add a vertex to the graph
    void addVertex(int v);

    // Add an edge to the graph
    void addEdge(int v1, int v2, int weight);

    // Perform a breadth-first search starting at the given vertex
    Graph bfs(int start);

    // Perform a depth-first search starting at the given vertex
    Graph dfs(int start);

    // Build a minimum spanning tree using Prim's algorithm
    std::vector<std::pair<int, int>> prim();

    

private:
    std::unordered_map<int, std::vector<std::pair<int, int>>> adjacencyList;
};

#endif // GRAPH_H
