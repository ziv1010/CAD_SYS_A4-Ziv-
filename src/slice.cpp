#include "slice.h"
#include <cmath>
#include <iostream>

// Helper function to check if a point lies between two other points in 3D
bool isBetween(float t) {
    return t >= 0.0f && t <= 1.0f;
}

// Function to check for intersection between an edge and the slicing plane
bool Plane::intersect(const Vertex& p1, const Vertex& p2, Vertex& intersection) const {
    // Parametrize the line r(t) = p1 + t * (p2 - p1)
    float x1 = p1.getX(), y1 = p1.getY(), z1 = p1.getZ();
    float x2 = p2.getX(), y2 = p2.getY(), z2 = p2.getZ();

    // Compute the denominator of the t equation
    float denominator = A * (x2 - x1) + B * (y2 - y1) + C * (z2 - z1);

    // If denominator is zero, the line is parallel to the plane
    if (std::abs(denominator) < 1e-6) {
        return false; // No intersection
    }

    // Solve for t
    float t = (D - (A * x1 + B * y1 + C * z1)) / denominator;

    // Check if the intersection is within the edge segment
    if (isBetween(t)) {
        intersection.setX(x1 + t * (x2 - x1));
        intersection.setY(y1 + t * (y2 - y1));
        intersection.setZ(z1 + t * (z2 - z1));
        return true;
    }

    return false;
}

// Function to slice the object into two parts across the given plane
void sliceObject(const Object3D& object, const Plane& plane, Object3D& leftSide, Object3D& rightSide) {
    // Map to keep track of new vertices created at intersections
    std::vector<Vertex> newVertices;

    for (const auto& edge : object.edges) {
        const Vertex& p1 = object.vertices[edge.v1_index];
        const Vertex& p2 = object.vertices[edge.v2_index];

        Vertex intersection(0, 0, 0);
        bool intersects = plane.intersect(p1, p2, intersection);

        if (intersects) {
            // Add the intersection vertex to both sides
            newVertices.push_back(intersection);

            leftSide.addVertex(intersection);
            rightSide.addVertex(intersection);
        }

        // Add vertices to the respective side based on their position relative to the plane
        if ((plane.A * p1.getX() + plane.B * p1.getY() + plane.C * p1.getZ() - plane.D) >= 0) {
            rightSide.addVertex(p1);
        } else {
            leftSide.addVertex(p1);
        }

        if ((plane.A * p2.getX() + plane.B * p2.getY() + plane.C * p2.getZ() - plane.D) >= 0) {
            rightSide.addVertex(p2);
        } else {
            leftSide.addVertex(p2);
        }
    }

    // Now update edges for both sides
    for (const auto& edge : object.edges) {
        const Vertex& p1 = object.vertices[edge.v1_index];
        const Vertex& p2 = object.vertices[edge.v2_index];

        Vertex intersection(0, 0, 0);
        bool intersects = plane.intersect(p1, p2, intersection);

        if (intersects) {
            // Split edge at the intersection and update both sides
            leftSide.addEdge(Edge(edge.v1_index, newVertices.size() - 1));
            rightSide.addEdge(Edge(newVertices.size() - 1, edge.v2_index));
        } else {
            // Keep original edge if no intersection
            if ((plane.A * p1.getX() + plane.B * p1.getY() + plane.C * p1.getZ() - plane.D) >= 0 &&
                (plane.A * p2.getX() + plane.B * p2.getY() + plane.C * p2.getZ() - plane.D) >= 0) {
                rightSide.addEdge(edge);
            } else {
                leftSide.addEdge(edge);
            }
        }
    }
}
