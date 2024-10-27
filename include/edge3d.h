#ifndef EDGE3D_H
#define EDGE3D_H

#include <cstdio>
#include <vector>   // Add this line
#include <string> 

enum class EdgeStatus {
    UNDECIDED,
    TRUE_EDGE,
    FALSE_EDGE
};

class Edge3D {
public:
    int startIdx;
    int endIdx;

    EdgeStatus status;
    std::vector<int> adjacentFaces; // Indices of faces adjacent to this edge

    Edge3D(int start, int end) : startIdx(start), endIdx(end), status(EdgeStatus::UNDECIDED) {}

    // Overloaded operators
    bool operator==(const Edge3D& other) const {
        return (startIdx == other.startIdx && endIdx == other.endIdx) ||
               (startIdx == other.endIdx && endIdx == other.startIdx);
    }

    // In edge3d.h
    std::string statusToString() const {
        switch (status) {
            case EdgeStatus::UNDECIDED: return "UNDECIDED";
            case EdgeStatus::TRUE_EDGE: return "TRUE";
            case EdgeStatus::FALSE_EDGE: return "FALSE";
            default: return "UNKNOWN";
        }
    }


    // Debug print
    void print() const {
        printf("Edge3D from Vertex %d to Vertex %d\n", startIdx, endIdx);
    }
};

#endif // EDGE3D_H