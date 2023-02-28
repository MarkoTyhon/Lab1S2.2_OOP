#include <iostream>
#include "Graph.h"
#include <stdio.h>
#include <fstream>


int main() {
    Graph g(6); // Create graph with 6 vertices
    g.addEdge(0, 1, 16); // Add edges with capacities
    g.addEdge(0, 2, 13);
    g.addEdge(1, 2, 10);
    g.addEdge(1, 3, 12);
    g.addEdge(2, 1, 4);
    g.addEdge(2, 4, 14);
    g.addEdge(3, 2, 9);
    g.addEdge(3, 5, 20);
    g.addEdge(4, 3, 7);
    g.addEdge(4, 5, 4);

    int source = 0;
    int sink = 5;

    std::cout << g.fordFulkerson(source, sink).sendToPython() << "\n";
}
    /*// Perform a breadth-first search starting at vertex 0
    std::cout << "BFS starting at vertex 0: ";

    std::vector<int> res = g.bfs(0);
    for (auto i: res){
        std::cout << i;
    }

    std::cout << "\n";

    // Perform a depth-first search starting at vertex 0
    std::cout << "DFS starting at vertex 0: ";

    std::vector<int> res2 = g.dfs(0);
    for (auto i: res2){
        std::cout << i;
    }

    std::cout << "\n";

    // Build a minimum spanning tree using Prim's algorithm
    std::vector<std::pair<int, int>> mst = g.prim();

    // Print the edges in the minimum spanning tree
    std::cout << "Minimum spanning tree:" << std::endl;
    for (const auto& edge : mst) {
        std::cout << edge.first << " - " << edge.second << std::endl;
    }

    std::vector<int> dist = g.dijkstra(0);

    std::cout << "Shortest path distances from vertex " << 0 << ":" << std::endl;
    for (int i = 0; i < dist.size(); i++) {
        std::cout << i << ": " << dist[i] << std::endl;
    }

    return 0;*/
    
    /*const int  MAX_SIZE = 256;
    char data[MAX_SIZE+1] = {0};

    int status;
    FILE *fp; 
    fp = _popen("python draw.py", "w");
    if (fp  != NULL){
        status = _pclose(fp);
        std::cout << status << "\n";
    }
    */

