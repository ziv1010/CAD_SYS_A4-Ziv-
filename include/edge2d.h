#ifndef EDGE2D_H
#define EDGE2D_H

class Edge2D {
public:
    int start_vertex_index;
    int end_vertex_index;

    Edge2D(int start_idx, int end_idx);
};

#endif // EDGE2D_H