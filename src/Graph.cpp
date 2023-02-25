#include "Graph.h"

void Graph::addVertex(int v) {
    adjacencyList[v] = {};
}

void Graph::addEdge(int v1, int v2, int weight) {
    adjacencyList[v1].emplace_back(v2, weight);
    adjacencyList[v2].emplace_back(v1, weight);
}

void Graph::bfs(int start) {
    std::queue<int> q;
    std::set<int> visited;

    q.push(start);
    visited.insert(start);

    while (!q.empty()) {
        int v = q.front();
        q.pop();

        // Do something with vertex v
        // ...

        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;

            if (!visited.count(u)) {
                q.push(u);
                visited.insert(u);
            }
        }
    }
}

void Graph::dfs(int start) {
    std::stack<int> s;
    std::set<int> visited;

    s.push(start);
    visited.insert(start);

    while (!s.empty()) {
        int v = s.top();
        s.pop();

        // Do something with vertex v
        // ...

        for (const auto& neighbor : adjacencyList[v]) {
            int u = neighbor.first;

            if (!visited.count(u)) {
                s.push(u);
                visited.insert(u);
            }
        }
    }
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
