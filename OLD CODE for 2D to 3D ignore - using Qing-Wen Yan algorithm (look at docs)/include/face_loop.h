// face_loop.h
#ifndef FACE_LOOP_H
#define FACE_LOOP_H

#include <vector>
#include <string>
#include "vector3d.h"   // Assuming Vector3D is defined in this header
#include "plane.h"      // Assuming Plane is defined in this header
#include "vertex3d.h"   // Assuming Vertex3D is defined in this header
#include "edge3d.h"     // Assuming Edge3D is defined in this header

class FaceLoop {
public:
    std::vector<Vertex3D> vertices; // Ordered list of vertices defining the face loop
    int sideSelection;              // +1 for positive side, -1 for negative side

    // Constructors
    FaceLoop() : sideSelection(1) {}
    FaceLoop(const std::vector<Vertex3D>& verts, int side = 1)
        : vertices(verts), sideSelection(side) {}

    // Methods
    Vector3D getNormal() const;
    Plane getPlane() const;
    bool containsPoint(const Vector3D& point) const;
    bool sharesEdgeWith(const FaceLoop& other, Edge3D& sharedEdge) const;
    std::string getUniqueIdentifier() const;

    // Equality operator for comparison
    bool operator==(const FaceLoop& other) const {
        return vertices == other.vertices && sideSelection == other.sideSelection;
    }
};

#endif // FACE_LOOP_H
