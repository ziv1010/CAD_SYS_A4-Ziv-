#ifndef EDGE2D_H
#define EDGE2D_H

#include "vertex2d.h"

class Edge2D {
public:
    int v1_index, v2_index; // Indices of vertices in the projection's vertex list
    bool isBroken;          // Depth information (broken or solid)
    bool isComplex;         // Edge type (simple or complex)

    Edge2D(int v1, int v2, bool isBroken = false, bool isComplex = false)
        : v1_index(v1), v2_index(v2), isBroken(isBroken), isComplex(isComplex) {}
};

#endif // EDGE2D_H
