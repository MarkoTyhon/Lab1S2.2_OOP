#include "Graph.h"


Graph::Graph(int V){
    for (int vetex = 0; vetex < V; vetex++){
        addVertex(vetex);
    }
}

void Graph::addVertex(int v) {
    adjacencyList[v] = {};
}


void Graph::addEdge(int v1, int v2, int weight) {
    adjacencyList[v1].emplace_back(v2, weight);
    adjacencyList[v2].emplace_back(v1, weight);
}

int Graph::getNumOfVertex(){
    return adjacencyList.size();
}

int Graph::getWeightOfEdge(int v1, int v2){
    for (auto& pair : adjacencyList[v1]){
        if (pair.first == v2){
            return pair.second;
        }
    }
    return -1;
}

Graph Graph::bfs(int start) {
    std::queue<int> q;
    std::vector<int> visited;
    Graph H(getNumOfVertex());

    q.push(start);
    visited.push_back(start);

    while (!q.empty()) {
        int v = q.front();
        q.pop();

        // Do something with vertex v
        // ...

        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;

            if (!std::count(visited.begin(),visited.end(), u)) {
                q.push(u);
                visited.push_back(u);
                H.addEdge(v, u, getWeightOfEdge(v, u));
            }
        }
    }
    return H;
}

Graph Graph::dfs(int start) {
    std::stack<int> s;
    std::vector<int> visited;
    Graph H(getNumOfVertex());

    s.push(start);
    visited.push_back(start);

    while (!s.empty()) {
        int v = s.top();
        s.pop();

        // Do something with vertex v
        // ...

        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;

            if (!std::count(visited.begin(), visited.end(), u)) {
                s.push(u);
                visited.push_back(u);
                H.addEdge(v, u, getWeightOfEdge(v, u));
            }
        }
    }
    return H;
}

Graph Graph::primMST(int start) {
    Graph MST(getNumOfVertex()); // create a new Graph to store the MST
    std::vector<int> key(getNumOfVertex(), INT_MAX); // key values used to pick minimum weight edge in cut
    std::vector<bool> mstSet(getNumOfVertex(), false); // to represent set of vertices not yet included in MST
    std::vector<int> parent(getNumOfVertex(), -1); // to store constructed MST
    key[start] = 0; // make key 0 for the starting vertex
    for (int count = 0; count < getNumOfVertex() - 1; ++count) {
        int u = -1;
        // Pick the minimum key vertex from the set of vertices not yet included in MST
        for (int v = 0; v < getNumOfVertex(); ++v) {
            if (!mstSet[v] && (u == -1 || key[v] < key[u])) {
                u = v;
            }
        }
        // Add the picked vertex to the MST Set
        mstSet[u] = true;
        // Update key and parent for adjacent vertices of the picked vertex
        for (auto const &pair : adjacencyList[u]) {
            int v = pair.first;
            int weight = pair.second;
            if (!mstSet[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
            }
        }
    }
    // Add edges of the MST to the new Graph
    for (int v = 1; v < getNumOfVertex(); ++v) {
        MST.addEdge(parent[v], v, key[v]);
    }
    return MST;
}

Graph Graph::fordFulkerson(int source, int sink) {
    // Create a residual graph with the same edges as the original graph
    Graph residual(*this);

    // Initialize parent vector to keep track of augmenting path
    std::vector<int> parent(getNumOfVertex(), -1);

    // Initialize maximum flow to zero
    int maxFlow = 0;

    // Loop until there is no more augmenting path from source to sink
    while (residual.hasAugmentingPath(source, sink, parent)) {
        // Find the maximum flow that can be sent along the augmenting path
        int pathFlow = INT_MAX;
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            int capacity = residual.getWeightOfEdge(u, v);
            pathFlow = std::min(pathFlow, capacity);
        }

        // Update the residual capacities of the edges along the augmenting path
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residual.setWeightOfEdge(u, v, residual.getWeightOfEdge(u, v) - pathFlow);
            residual.setWeightOfEdge(v, u, residual.getWeightOfEdge(v, u) + pathFlow);
        }

        // Add the flow along the augmenting path to the maximum flow
        maxFlow += pathFlow;
    }

    // Create a new graph with the same vertices and edges as the original graph,
    // but with the capacities of the edges set to the amount of flow sent along them
    Graph result(getNumOfVertex());
    for (int v = 0; v < getNumOfVertex(); v++) {
        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;
            int capacity = neighbor.second;
            int flow = capacity - residual.getWeightOfEdge(v, u);
            result.addEdge(v, u, flow);
        }
    }

    return result;
}

/*bool Graph::hasAugmentingPath(int source, int sink, std::vector<int>& parent) {
    // Initialize visited vector to keep track of visited vertices
    std::vector<bool> visited(getNumOfVertex(), false);

    // Initialize queue for BFS
    std::queue<int> q;

    // Start BFS from source vertex
    q.push(source);
    visited[source] = true;

    // Loop until we reach the sink or the queue is empty
    while (!q.empty()) {
        int v = q.front();
        q.pop();

        // Check if we have reached the sink vertex
        if (v == sink) {
            return true;
        }

        // Check all neighbors of v that have residual capacity and haven't been visited yet
        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;
            int capacity = neighbor.second;

            if (!visited[u] && capacity > 0) {
                q.push(u);
                visited[u] = true;
                parent[u] = v;
            }
        }
    }

    // If we haven't found an augmenting path, return false
    return false;
}

void Graph::setWeightOfEdge(int v1, int v2, int weight) {
    // Check if the edge already exists in the adjacency list
    for (auto& pair : adjacencyList[v1]) {
        if (pair.first == v2) {
            pair.second = weight;
            return;
        }
    }
    // If the edge doesn't exist, add it to the adjacency list
    adjacencyList[v1].emplace_back(v2, weight);
}*/

int Graph::sendToPython(){
    std::string stringGraph = "python draw.py " + std::to_string(getNumOfVertex()) + " ";

    for (auto const & keyEndVal : adjacencyList) {
        for (auto const & elem : keyEndVal.second){
            stringGraph += std::to_string(keyEndVal.first) + ":" + std::to_string(elem.first) + ":" + std::to_string(elem.second)+ " ";
        }
	} 

    std::cout << stringGraph << "\n";
    int size = stringGraph.size();
    char* data = new char[size+1];

    strcpy(data, stringGraph.c_str());

    int status;
    FILE *fp; 
    fp = _popen(data, "w");
    if (fp  != NULL){
        status = _pclose(fp);
        std::cout << status << "\n";
    }

    delete[] data;

	return status;
} 


std::ostream& operator << (std::ostream& os ,const Graph&  obj){
	for (auto const & keyEndVal : obj.adjacencyList) {
		os << keyEndVal.first << " :";
        for (auto const & elem : keyEndVal.second){ 
            os << " (" << elem.first << ", " << elem.second << ")";
        }
		os << ";\n";
	} 
	return os;
}