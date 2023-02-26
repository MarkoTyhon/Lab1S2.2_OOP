#include "Graph.h"

void Graph::addVertex(int v) {
    adjacencyList[v] = {};
}

void Graph::addEdge(int v1, int v2, int weight) {
    adjacencyList[v1].emplace_back(v2, weight);
    adjacencyList[v2].emplace_back(v1, weight);
}

std::vector<int> Graph::bfs(int start) {
    std::queue<int> q;
    std::vector<int> visited;

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
            }
        }
    }
    return visited;
}

std::vector<int> Graph::dfs(int start) {
    std::stack<int> s;
    std::vector<int> visited;

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
            }
        }
    }
    return visited;
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

std::vector<int> Graph::dijkstra(int start){
    // Set up data structures for Dijkstra's algorithm
    std::vector<int> dist(adjacencyList.size(), std::numeric_limits<int>::max());
    std::set<int> visited;
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<>> pq;
    pq.push(std::make_pair(0, start));
    dist[start] = 0;

    // Run Dijkstra's algorithm
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        visited.insert(u);
        for (const auto& neighbor : adjacencyList.at(u)) {
            int v = neighbor.first;
            int weight = neighbor.second;
            if (visited.find(v) == visited.end() && dist[u] != std::numeric_limits<int>::max() && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push(std::make_pair(dist[v], v));
            }
        }
    }

    return dist;
}
