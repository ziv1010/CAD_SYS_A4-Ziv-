#ifndef EDGE3D_H
#define EDGE3D_H

#include <cstdio>

class Edge3D {
public:
    int startIdx;
    int endIdx;

    // Constructors
    Edge3D() : startIdx(0), endIdx(0) {}
    Edge3D(int s, int e) : startIdx(s), endIdx(e) {}

    // Overloaded operators
    bool operator==(const Edge3D& other) const {
        return (startIdx == other.startIdx && endIdx == other.endIdx) ||
               (startIdx == other.endIdx && endIdx == other.startIdx);
    }

    // Debug print
    void print() const {
        printf("Edge3D from Vertex %d to Vertex %d\n", startIdx, endIdx);
    }
};

#endif // EDGE3D_H