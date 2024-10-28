// face_loop.cpp
#include "face_loop.h"
#include <sstream>
#include <cmath>

// Implement getNormal()
Vector3D FaceLoop::getNormal() const {
    if (vertices.size() < 3) {
        return Vector3D(0.0f, 0.0f, 0.0f); // Invalid face loop
    }

    Vector3D edge1 = vertices[1].position - vertices[0].position;
    Vector3D edge2 = vertices[2].position - vertices[1].position;

    Vector3D normal = edge1.cross(edge2).normalize();
    return normal * sideSelection; // Adjust based on side selection
}

// Implement getPlane()
Plane FaceLoop::getPlane() const {
    Vector3D normal = getNormal();
    float d = normal.dot(vertices[0].position);
    return Plane(normal, d);
}

// Implement containsPoint() using ray casting or other point-in-polygon algorithms
bool FaceLoop::containsPoint(const Vector3D& point) const {
    // Simplified 2D point-in-polygon assuming projection onto plane
    // Implement the actual logic based on the plane's orientation
    // Placeholder implementation
    return false;
}

// Implement sharesEdgeWith()
bool FaceLoop::sharesEdgeWith(const FaceLoop& other, Edge3D& sharedEdge) const {
    for (size_t i = 0; i < vertices.size(); ++i) {
        size_t next = (i + 1) % vertices.size();
        Edge3D edge1(vertices[i].index, vertices[next].index);

        for (size_t j = 0; j < other.vertices.size(); ++j) {
            size_t otherNext = (j + 1) % other.vertices.size();
            Edge3D edge2(other.vertices[j].index, other.vertices[otherNext].index);

            if (edge1 == edge2) {
                sharedEdge = edge1;
                return true;
            }
        }
    }
    return false;
}

// Implement getUniqueIdentifier()
std::string FaceLoop::getUniqueIdentifier() const {
    std::stringstream ss;
    for (const auto& vertex : vertices) {
        ss << vertex.index << "-";
    }
    ss << sideSelection;
    return ss.str();
}
