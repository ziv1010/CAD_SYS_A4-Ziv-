#ifndef EDGE3D_H
#define EDGE3D_H

class Edge3D {
public:
    int v1_index, v2_index; // Indices of vertices forming the edge
    bool isValid; // Indicates if the edge is valid
    Edge3D(int v1 = -1, int v2 = -1) : v1_index(v1), v2_index(v2) {}
};

#endif // EDGE3D_H
