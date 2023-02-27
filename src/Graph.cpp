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

std::vector<std::pair<int, int>> Graph::prim() {
    std::set<int> visited;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    std::vector<std::pair<int, int>> mst;

    // Start with an arbitrary vertex
    int start = adjacencyList.begin()->first;
    visited.insert(start);

    // Add all edges connected to the starting vertex to the priority queue
    for (const auto& neighbor : adjacencyList[start]) {
        pq.emplace(neighbor.second, neighbor.first);
    }

    while (!pq.empty()) {
        auto edge = pq.top();
        pq.pop();

        int weight = edge.first;
        int v = edge.second;

        if (visited.count(v)) {
            continue;
        }

        visited.insert(v);
        mst.emplace_back(start, v);

        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;
            int weight = neighbor.second;

            if (!visited.count(u)) {
                pq.emplace(weight, u);
            }
        }
    }

    return mst;
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