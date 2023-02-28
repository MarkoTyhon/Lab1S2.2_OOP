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



int Graph::fordFulkerson(int source, int sink) {
    // Initialize residual graph
    std::vector<std::vector<int>> residual(getNumOfVertex(), std::vector<int>(getNumOfVertex(), 0));
    for (int v = 0; v < getNumOfVertex(); v++) {
        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;
            int weight = neighbor.second;
            residual[v][u] = weight;
        }
    }

    int maxFlow = 0;
    while (true) {
        // Find an augmenting path in the residual graph
        std::vector<int> parent(getNumOfVertex(), -1);
        std::queue<int> q;
        q.push(source);
        while (!q.empty()) {
            int v = q.front();
            q.pop();
            for (int u = 0; u < getNumOfVertex(); u++) {
                if (parent[u] == -1 && residual[v][u] > 0) {
                    parent[u] = v;
                    q.push(u);
                }
            }
        }
        if (parent[sink] == -1) break;

        // Compute the augmenting flow and update the residual graph
        int augmentingFlow = INT_MAX;
        for (int u = sink; u != source; u = parent[u]) {
            int v = parent[u];
            augmentingFlow = std::min(augmentingFlow, residual[v][u]);
        }
        for (int u = sink; u != source; u = parent[u]) {
            int v = parent[u];
            residual[v][u] -= augmentingFlow;
            residual[u][v] += augmentingFlow;
        }
        maxFlow += augmentingFlow;
    }

    return maxFlow;
}

int Graph::findEdgeIndex(int u, int v) {
    for (int i = 0; i < adjacencyList[u].size(); i++) {
        if (adjacencyList[u][i].first == v) {
            return i;
        }
    }
    return -1;
}

Graph Graph::dijkstra(int start) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    std::vector<int> dist(getNumOfVertex(), std::numeric_limits<int>::max());
    std::vector<int> prev(getNumOfVertex(), -1);
    Graph H(getNumOfVertex());

    pq.push(std::make_pair(0, start));
    dist[start] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const auto& neighbor : adjacencyList[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                prev[v] = u;
                pq.push(std::make_pair(dist[v], v));
            }
        }
    }

    for (int i = 0; i < getNumOfVertex(); i++) {
        if (i != start && prev[i] != -1) {
            H.addEdge(i, prev[i], getWeightOfEdge(i, prev[i]));
        }
    }

    return H;
}

Graph Graph::aStar(int start, int goal) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    std::vector<int> dist(getNumOfVertex(), std::numeric_limits<int>::max());
    std::vector<int> prev(getNumOfVertex(), -1);
    Graph H(getNumOfVertex());

    pq.push(std::make_pair(0, start));
    dist[start] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == goal) {
            break;
        }

        for (const auto& neighbor : adjacencyList[u]) {
            int v = neighbor.first;
            int weight = neighbor.second;

            int heuristic = std::abs(v - goal);

            if (dist[u] + weight + heuristic < dist[v]) {
                dist[v] = dist[u] + weight + heuristic;
                prev[v] = u;
                pq.push(std::make_pair(dist[v], v));
            }
        }
    }

    for (int i = 0; i < getNumOfVertex(); i++) {
        if (i != start && prev[i] != -1) {
            H.addEdge(i, prev[i], getWeightOfEdge(i, prev[i]));
        }
    }

    return H;
}


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