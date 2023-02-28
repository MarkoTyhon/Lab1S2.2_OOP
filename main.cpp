#include <iostream>
#include "Graph.h"
#include <stdio.h>
#include <fstream>


int main() {
    // Create a graph with 4 vertices and 5 edges
    Graph g(6);
    g.addEdge(0, 1, 8);
    g.addEdge(0, 4, 3);
    g.addEdge(1, 2, 9);
    g.addEdge(2, 4, 7);
    g.addEdge(2, 5, 2);
    g.addEdge(3, 5, 5);
    g.addEdge(4, 2, 7);
    g.addEdge(4, 3, 4);

    g.sendToPython();
    g.bfs(0).sendToPython();
    g.dfs(0).sendToPython();
    g.dijkstra(0).sendToPython();
    g.aStar(0,4).sendToPython();    
    std::cout << "MaxFlow: " << g.fordFulkerson(0,4) << "\n";
}
   