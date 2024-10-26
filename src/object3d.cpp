#include "object3d.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <map>
#include <utility>
#include <iostream>

// Project an edge onto a specified plane
std::pair<Vertex, Vertex> Object3D::projectEdgeToPlane(const Edge& edge, const std::string& plane) const {
    const Vertex& v1 = vertices[edge.v1_index];
    const Vertex& v2 = vertices[edge.v2_index];

    if (plane == "XY") {
        return { Vertex(v1.getX(), v1.getY(), 0), Vertex(v2.getX(), v2.getY(), 0) };
    } else if (plane == "XZ") {
        return { Vertex(v1.getX(), 0, v1.getZ()), Vertex(v2.getX(), 0, v2.getZ()) };
    } else { // "YZ"
        return { Vertex(0, v1.getY(), v1.getZ()), Vertex(0, v2.getY(), v2.getZ()) };
    }
}

// Check if two 2D edges intersect and return the intersection point
bool Object3D::edgesIntersect2D(const Vertex& p1, const Vertex& p2,
                                const Vertex& q1, const Vertex& q2,
                                Vertex& intersection) const {
    // Convert vertices to 2D points (x, y)
    float x1 = p1.getX(), y1 = p1.getY();
    float x2 = p2.getX(), y2 = p2.getY();
    float x3 = q1.getX(), y3 = q1.getY();
    float x4 = q2.getX(), y4 = q2.getY();

    // Compute denominators
    float denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1);

    if (fabs(denom) < 1e-6) {
        // Lines are parallel or coincident
        // Check for overlapping segments
        // Calculate orientation


        // Simple overlap detection in 1D
        auto onSegment = [&](float xi1, float xi2, float xk) -> bool {
            return std::min(xi1, xi2) <= xk + 1e-6 && xk <= std::max(xi1, xi2) + 1e-6;
        };

        // Check if the projections overlap
        bool overlapX = onSegment(x1, x2, x3) || onSegment(x1, x2, x4) ||
                        onSegment(x3, x4, x1) || onSegment(x3, x4, x2);
        bool overlapY = onSegment(y1, y2, y3) || onSegment(y1, y2, y4) ||
                        onSegment(y3, y4, y1) || onSegment(y3, y4, y2);

        if (overlapX && overlapY) {
            // Overlapping segments
            // For simplicity, we'll consider overlapping edges as intersecting at multiple points
            // This implementation can be expanded based on specific requirements
            return false; // Not handling overlapping in this function
        }

        return false;
    }

    float ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) / denom;
    float ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3)) / denom;

    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
        // Intersection point
        float x = x1 + ua * (x2 - x1);
        float y = y1 + ua * (y2 - y1);
        intersection.setX(x);
        intersection.setY(y);
        intersection.setZ(0); // Since it's 2D projection
        return true;
    }

    return false;
}

// Compute the corresponding 3D intersection point based on the projection plane
Vertex Object3D::compute3DIntersectionPoint(const Edge& edge, const Vertex& intersection2D, const std::string& plane) const {
    const Vertex& v1 = vertices[edge.v1_index];
    const Vertex& v2 = vertices[edge.v2_index];

    // Calculate parameter t for interpolation
    float t = 0.0f;
    if (plane == "XY") {
        float dx = v2.getX() - v1.getX();
        float dy = v2.getY() - v1.getY();
        if (fabs(dx) > fabs(dy)) {
            t = (intersection2D.getX() - v1.getX()) / dx;
        } else {
            t = (intersection2D.getY() - v1.getY()) / dy;
        }
    } else if (plane == "XZ") {
        float dx = v2.getX() - v1.getX();
        float dz = v2.getZ() - v1.getZ();
        if (fabs(dx) > fabs(dz)) {
            t = (intersection2D.getX() - v1.getX()) / dx;
        } else {
            t = (intersection2D.getZ() - v1.getZ()) / dz;
        }
    } else { // "YZ"
        float dy = v2.getY() - v1.getY();
        float dz = v2.getZ() - v1.getZ();
        if (fabs(dy) > fabs(dz)) {
            t = (intersection2D.getY() - v1.getY()) / dy;
        } else {
            t = (intersection2D.getZ() - v1.getZ()) / dz;
        }
    }

    // Clamp t to [0,1] to avoid numerical issues
    t = std::max(0.0f, std::min(1.0f, t));

    // Compute the 3D intersection point
    float x = v1.getX() + t * (v2.getX() - v1.getX());
    float y = v1.getY() + t * (v2.getY() - v1.getY());
    float z = v1.getZ() + t * (v2.getZ() - v1.getZ());

    return Vertex(x, y, z);
}

// Process edge intersections across all projection planes
void Object3D::processEdgeIntersections() {
    // Create a copy of edges to iterate over
    std::vector<Edge> S = edges;
    std::vector<Edge> updatedEdges;
    std::vector<Vertex> newVertices = vertices;

    // Define projection planes
    std::vector<std::string> planes = {"XY", "XZ", "YZ"};

    // Set to keep track of edges to remove (using their indices)
    std::set<size_t> edgesToRemove;

    // Iterate over each projection plane
    for (const auto& plane : planes) {
        // Iterate over all unique pairs of edges
        for (size_t i = 0; i < S.size(); ++i) {
            for (size_t j = i + 1; j < S.size(); ++j) {
                // Skip if edges are already processed
                if (edgesToRemove.find(i) != edgesToRemove.end() ||
                    edgesToRemove.find(j) != edgesToRemove.end()) {
                    continue;
                }

                const Edge& e_i = S[i];
                const Edge& e_j = S[j];

                // Project edges onto the current plane
                std::pair<Vertex, Vertex> proj_ei = projectEdgeToPlane(e_i, plane);
                std::pair<Vertex, Vertex> proj_ej = projectEdgeToPlane(e_j, plane);

                // Check for intersection
                Vertex intersection2D(0.0f, 0.0f, 0.0f);
                bool intersect = edgesIntersect2D(proj_ei.first, proj_ei.second,
                                                 proj_ej.first, proj_ej.second,
                                                 intersection2D);

                if (intersect) {
                    // Compute corresponding 3D intersection points
                    Vertex p_i = compute3DIntersectionPoint(e_i, intersection2D, plane);
                    Vertex p_j = compute3DIntersectionPoint(e_j, intersection2D, plane);

                    // Add new vertices to the list
                    size_t index_p_i = newVertices.size();
                    newVertices.push_back(p_i);
                    size_t index_p_j = newVertices.size();
                    newVertices.push_back(p_j);

                    // Split edge e_i into (v1, p_i) and (p_i, v2)
                    updatedEdges.emplace_back(e_i.v1_index, index_p_i);
                    updatedEdges.emplace_back(index_p_i, e_i.v2_index);

                    // Split edge e_j into (v1, p_j) and (p_j, v2)
                    updatedEdges.emplace_back(e_j.v1_index, index_p_j);
                    updatedEdges.emplace_back(index_p_j, e_j.v2_index);

                    // Mark original edges for removal
                    edgesToRemove.insert(i);
                    edgesToRemove.insert(j);
                }

                // Handle overlapping segments if necessary
                // (This implementation does not handle overlapping; you can extend it as needed)
            }
        }
    }

    // Remove marked edges and add updated edges
    std::vector<Edge> finalEdges;
    for (size_t i = 0; i < S.size(); ++i) {
        if (edgesToRemove.find(i) == edgesToRemove.end()) {
            finalEdges.push_back(S[i]);
        }
    }

    // Add the newly split edges
    finalEdges.insert(finalEdges.end(), updatedEdges.begin(), updatedEdges.end());

    // Update the object's vertices and edges
    vertices = newVertices;
    edges = finalEdges;
}


// Helper function to compute the area of a triangle
// **Add this function at the top of object3d.cpp, before computeSurfaceArea**
float triangleArea(const Vertex& v0, const Vertex& v1, const Vertex& v2) {
    Vertex edge1 = v1 - v0;
    Vertex edge2 = v2 - v0;
    Vertex cross = edge1.crossProduct(edge2);
    return 0.5f * cross.length();
}

// Compute the total surface area
float Object3D::computeSurfaceArea() const {
    float totalArea = 0.0f;

    for (const auto& face : faces) {
        const std::vector<int>& indices = face.getVertexIndices();
        if (indices.size() < 3) continue; // Skip degenerate faces

        std::vector<Vertex> faceVertices;
        for (int idx : indices) {
            faceVertices.push_back(vertices[idx]);
        }

        // Compute face normal
        Vertex normal = (faceVertices[1] - faceVertices[0]).crossProduct(faceVertices[2] - faceVertices[0]);
        normal.normalize();

        const Vertex& v0 = faceVertices[0];
        for (size_t i = 1; i + 1 < faceVertices.size(); ++i) {
            const Vertex& v1 = faceVertices[i];
            const Vertex& v2 = faceVertices[i + 1];

            // Compute area of the triangle
            float area = triangleArea(v0, v1, v2);

            // Determine orientation
            Vertex triNormal = (v1 - v0).crossProduct(v2 - v0);
            float orientation = normal.dotProduct(triNormal);

            if (orientation < 0) {
                area = -area; // Invert area if orientation is opposite
            }

            totalArea += area;
        }
    }

    return totalArea;
}

float Object3D::computeVolume() const {
    float totalVolume = 0.0f;

    for (const auto& face : faces) {
        const std::vector<int>& indices = face.getVertexIndices();
        if (indices.size() < 3) continue;

        std::vector<Vertex> faceVertices;
        for (int idx : indices) {
            faceVertices.push_back(vertices[idx]);
        }

        const Vertex& v0 = faceVertices[0];
        for (size_t i = 1; i + 1 < faceVertices.size(); ++i) {
            const Vertex& v1 = faceVertices[i];
            const Vertex& v2 = faceVertices[i + 1];

            float volume = v0.dotProduct(v1.crossProduct(v2)) / 6.0f;
            totalVolume += volume;
        }
    }

    return std::abs(totalVolume);
}

