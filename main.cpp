#include <iostream>
#include "Graph.h"

int main() {
    // Create a new graph
    Graph g;

    // Add vertices to the graph
    g.addVertex(0);
    g.addVertex(1);
    g.addVertex(2);
    g.addVertex(3);
    g.addVertex(4);

    // Add edges to the graph
    g.addEdge(0, 1, 2);
    g.addEdge(0, 3, 1);
    g.addEdge(1, 2, 3);
    g.addEdge(1, 3, 2);
    g.addEdge(1, 4, 4);
    g.addEdge(2, 4, 5);
    g.addEdge(3, 4, 1);

    // Perform a breadth-first search starting at vertex 0
    std::cout << "BFS starting at vertex 0: ";
    g.bfs(0);
    std::cout << std::endl;

    // Perform a depth-first search starting at vertex 0
    std::cout << "DFS starting at vertex 0: ";
    g.dfs(0);
    std::cout << std::endl;

    // Build a minimum spanning tree using Prim's algorithm
    std::vector<std::pair<int, int>> mst = g.prim();

    // Print the edges in the minimum spanning tree
    std::cout << "Minimum spanning tree:" << std::endl;
    for (const auto& edge : mst) {
        std::cout << edge.first << " - " << edge.second << std::endl;
    }

    return 0;
}