// plane.h

#ifndef PLANE_H
#define PLANE_H

#include <vector>
#include "edge3d.h"

class Plane {
public:
    float a, b, c, d; // Plane equation coefficients: ax + by + cz + d = 0
    std::vector<int> edgesOnPlane; // Indices of edges lying on this plane

    // New member variables
    std::vector<std::vector<int>> basicLoops; // Each basic loop is a list of edge indices
    std::vector<std::vector<int>> faceLoops;  // Face loops formed from basic loops

    // Constructor
    Plane(float a_ = 0.0f, float b_ = 0.0f, float c_ = 0.0f, float d_ = 0.0f)
        : a(a_), b(b_), c(c_), d(d_) {}

    // Method to compare two planes within a tolerance
    bool isSimilar(const Plane& other, float tolerance = 1e-5f) const {
        float diff = std::sqrt(
            (a - other.a) * (a - other.a) +
            (b - other.b) * (b - other.b) +
            (c - other.c) * (c - other.c) +
            (d - other.d) * (d - other.d)
        );
        return diff <= tolerance;
    }
};

#endif // PLANE_H
