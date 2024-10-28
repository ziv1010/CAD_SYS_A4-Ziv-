#include "slicer.h"
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>

int Slicer::classifyVertex(const Vertex& v, const Plane& plane) {
    float d = plane.normal.dotProduct(v - plane.point);
    if (fabs(d) < 1e-6) return 0; // On the plane
    return (d > 0) ? 1 : -1;
}

Vertex Slicer::computeIntersection(const Vertex& v1, const Vertex& v2, const Plane& plane) {
    Vertex direction = v2 - v1;
    float numerator = plane.normal.dotProduct(plane.point - v1);
    float denominator = plane.normal.dotProduct(direction);
    float t = numerator / denominator;
    return v1 + direction * t;
}

void Slicer::sliceObject(const Object3D& object, const Plane& plane, Object3D& objectAbove, Object3D& objectBelow) {
    // Maps from original vertex indices to new indices in the above and below objects
    std::map<int, int> vertexMapAbove;
    std::map<int, int> vertexMapBelow;

    // Maps for intersection vertices
    std::map<std::pair<int, int>, Vertex> intersectionVertices;
    std::map<std::pair<int, int>, int> intersectionVertexIndicesAbove;
    std::map<std::pair<int, int>, int> intersectionVertexIndicesBelow;

    // Classify all vertices
    std::vector<int> vertexClassifications(object.vertices.size());
    for (size_t i = 0; i < object.vertices.size(); ++i) {
        int classification = classifyVertex(object.vertices[i], plane);
        vertexClassifications[i] = classification;
        if (classification >= 0) {
            // Vertex is on or above the plane
            int newIndex = objectAbove.vertices.size();
            vertexMapAbove[i] = newIndex;
            objectAbove.vertices.push_back(object.vertices[i]);
        }
        if (classification <= 0) {
            // Vertex is on or below the plane
            int newIndex = objectBelow.vertices.size();
            vertexMapBelow[i] = newIndex;
            objectBelow.vertices.push_back(object.vertices[i]);
        }
    }

    // Process faces
    for (const auto& face : object.faces) {
        const std::vector<int>& indices = face.getVertexIndices();
        size_t n = indices.size();
        std::vector<int> classifications(n);
        for (size_t i = 0; i < n; ++i) {
            classifications[i] = vertexClassifications[indices[i]];
        }

        // Collect vertices for above and below objects
        std::vector<int> newFaceIndicesAbove;
        std::vector<int> newFaceIndicesBelow;

        for (size_t i = 0; i < n; ++i) {
            int idxCurrent = indices[i];
            int idxNext = indices[(i + 1) % n];
            int classCurrent = classifications[i];
            int classNext = classifications[(i + 1) % n];

            const Vertex& vCurrent = object.vertices[idxCurrent];
            const Vertex& vNext = object.vertices[idxNext];

            // For above object
            if (classCurrent >= 0) {
                // Current vertex is on or above the plane
                if (vertexMapAbove.count(idxCurrent)) {
                    newFaceIndicesAbove.push_back(vertexMapAbove[idxCurrent]);
                }
            }
            // For below object
            if (classCurrent <= 0) {
                if (vertexMapBelow.count(idxCurrent)) {
                    newFaceIndicesBelow.push_back(vertexMapBelow[idxCurrent]);
                }
            }

            // Check if edge crosses the plane
            if ((classCurrent > 0 && classNext < 0) || (classCurrent < 0 && classNext > 0)) {
                // Edge crosses the plane, compute intersection
                Vertex intersection = computeIntersection(vCurrent, vNext, plane);
                int newIndexAbove, newIndexBelow;

                // Use edgeKey as the key
                auto edgeKey = std::make_pair(std::min(idxCurrent, idxNext), std::max(idxCurrent, idxNext));

                if (intersectionVertices.count(edgeKey) == 0) {
                    intersectionVertices[edgeKey] = intersection;

                    newIndexAbove = objectAbove.vertices.size();
                    objectAbove.vertices.push_back(intersection);
                    intersectionVertexIndicesAbove[edgeKey] = newIndexAbove;

                    newIndexBelow = objectBelow.vertices.size();
                    objectBelow.vertices.push_back(intersection);
                    intersectionVertexIndicesBelow[edgeKey] = newIndexBelow;
                } else {
                    intersection = intersectionVertices[edgeKey];
                    newIndexAbove = intersectionVertexIndicesAbove[edgeKey];
                    newIndexBelow = intersectionVertexIndicesBelow[edgeKey];
                }

                // Add intersection vertex to faces
                if (classCurrent >= 0) {
                    newFaceIndicesAbove.push_back(newIndexAbove);
                }
                if (classCurrent <= 0) {
                    newFaceIndicesBelow.push_back(newIndexBelow);
                }
                if (classNext >= 0) {
                    newFaceIndicesAbove.push_back(newIndexAbove);
                }
                if (classNext <= 0) {
                    newFaceIndicesBelow.push_back(newIndexBelow);
                }
            }
        }

        // Build new faces
        if (newFaceIndicesAbove.size() >= 3) {
            // Remove duplicate vertices
            auto end = std::unique(newFaceIndicesAbove.begin(), newFaceIndicesAbove.end());
            newFaceIndicesAbove.erase(end, newFaceIndicesAbove.end());

            objectAbove.addFace(Face(newFaceIndicesAbove));
        }
        if (newFaceIndicesBelow.size() >= 3) {
            // Remove duplicate vertices
            auto end = std::unique(newFaceIndicesBelow.begin(), newFaceIndicesBelow.end());
            newFaceIndicesBelow.erase(end, newFaceIndicesBelow.end());

            objectBelow.addFace(Face(newFaceIndicesBelow));
        }
    }

    // Edges need to be reconstructed similarly
    // For simplicity, you can reconstruct edges from faces or skip them if not needed
}
