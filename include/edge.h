#ifndef EDGE_H
#define EDGE_H

class Edge {
public:
    int v1_index, v2_index;  // Indices of vertices forming an edge
    Edge(int v1, int v2) : v1_index(v1), v2_index(v2) {}
};

#endif