#ifndef SLICE_H
#define SLICE_H

#include "object3d.h"
#include <utility>
#include <vector>

class Plane {
public:
    float A, B, C, D; // Plane equation: Ax + By + Cz = D

    Plane(float A, float B, float C, float D) : A(A), B(B), C(C), D(D) {}

    // Given an edge (p1, p2), find the intersection with the plane, if any.
    bool intersect(const Vertex& p1, const Vertex& p2, Vertex& intersection) const;
};

// Function to slice an object across a given plane
void sliceObject(const Object3D& object, const Plane& plane, Object3D& leftSide, Object3D& rightSide);

#endif // SLICE_H
