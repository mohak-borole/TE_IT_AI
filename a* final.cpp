#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <cmath>

using namespace std;

#define INF INT_MAX

// Structure to represent a node in the graph
// struct Node {
//     int id;
//     int heuristic;
//     Node(int i, int h) : id(i), heuristic(h) {}
// };

// Structure to represent an edge in the graph
// struct Edge {
//     int to;
//     int weight;
//     Edge(int t, int w) : to(t), weight(w) {}
// };

// A* search algorithm implementation
vector<int> AStar(const vector<vector<int>>& graph, const vector<int>& heuristic, int start, int goal) {
    int n = graph.size();
    vector<int> distance(n, INF);
    vector<int> parent(n, -1);
    vector<bool> visited(n, false);

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // min-heap

    distance[start] = 0;
    pq.push({heuristic[start], start});

    while (!pq.empty()) {
        int curr = pq.top().second; //Prioritizing queue based on second value(Node id)
        pq.pop();

        if (curr == goal)
            break;

        if (visited[curr])
            continue;   //Checking loops

        visited[curr] = true;

        for (int i = 0; i < n; i++) {
            if (graph[curr][i] != INF) {
                int newDistance = distance[curr] + graph[curr][i];
                if (newDistance < distance[i]) {
                    distance[i] = newDistance;
                    parent[i] = curr;
                    pq.push({newDistance + heuristic[i], i});
                }
            }
        }
    }

    vector<int> path;
    for (int a = goal; a != -1; a = parent[a])
        path.push_back(a); //push_back() is used to add elements to a vector(Dynamic Array) at the end.

    reverse(path.begin(), path.end());

    return path;
}

int main() {
    // Example graph represented as an adjacency matrix
    vector<vector<int>> graph = {
        // A   B    C    D    E    F    G    H
        {  0 , 11 , 14 , 7  , INF, INF, INF, INF},  // A
        { 11 , 0  , INF, INF, 15 , INF, INF, INF},  // B
        { 14 , INF, 0  , INF,  8 , 10 , INF, INF},  // C
        {  7 , INF, INF,  0 , INF, 25 , INF, INF},  // D
        { INF, 15 , 8  , INF, 0  , INF, INF, 9  },  // E
        { INF, INF, 10 , 25 , INF,  0 , 20 , INF},  // F
        { INF, INF, INF, INF, INF, 20 ,  0 , 10 },  // G
        { INF, INF, INF, INF,  9 , INF, 10 , 0  }   // H
    };

    // Heuristic values for eac h node
    vector<int> heuristic = {40 , 32 , 25 , 35 , 19 , 17 , 0 , 10};

    int start = 0;
    int goal = 6;

    vector<int> path = AStar(graph, heuristic, start, goal);

    // Printing the shortest path
    cout << "Shortest path from node " << start << " to node " << goal << ": ";
    for (int node : path)
        cout << char(node + 65) << " ";
    cout << endl;

    return 0;
}
