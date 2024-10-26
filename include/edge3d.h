#ifndef EDGE3D_H
#define EDGE3D_H

class Edge3D {
public:
    int start_vertex_index;
    int end_vertex_index;
    bool examined; // For RER and PEVR procedures

    Edge3D(int start_idx, int end_idx);
};

#endif // EDGE3D_H
