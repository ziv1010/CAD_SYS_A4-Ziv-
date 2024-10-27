#include "graph2d.h"
#include <iostream>
#include <cmath>
#include <set>
#include <unordered_map>
#include <algorithm>

#define EPSILON 1e-6

void Graph2D::addVertex(const Point2D& point) {
    vertices.push_back(point);
}

void Graph2D::addEdge(const Line2D& line) {
    edges.push_back(line);
}

void Graph2D::removeDuplicateVertices() {
    std::vector<Point2D> uniqueVertices;
    std::unordered_map<size_t, size_t> indexMap; // Old index -> New index

    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto& v = vertices[i];
        auto it = std::find(uniqueVertices.begin(), uniqueVertices.end(), v);
        if (it == uniqueVertices.end()) {
            indexMap[i] = uniqueVertices.size();
            uniqueVertices.push_back(v);
        } else {
            indexMap[i] = std::distance(uniqueVertices.begin(), it);
        }
    }

    // Update edges to use new indices
    for (auto& edge : edges) {
        edge.startIdx = indexMap[edge.startIdx];
        edge.endIdx = indexMap[edge.endIdx];
    }

    vertices = uniqueVertices;
    std::cout << "[Debug][" << viewName << "] Removed duplicate vertices. Total vertices: " << vertices.size() << std::endl;
}

bool findIntersection(const Point2D& A1, const Point2D& A2, const Point2D& B1, const Point2D& B2, Point2D& intersection) {
    // Line A represented as A1 + s*(A2 - A1)
    // Line B represented as B1 + t*(B2 - B1)
    float x_K = A1.x, y_K = A1.y;
    float x_L = A2.x, y_L = A2.y;
    float x_M = B1.x, y_M = B1.y;
    float x_N = B2.x, y_N = B2.y;

    float denominator = (x_N - x_M)*(y_L - y_K) - (y_N - y_M)*(x_L - x_K);

    if (std::fabs(denominator) < EPSILON) {
        // Lines are parallel or coincident
        return false;
    }

    float s_J = ((x_N - x_M)*(y_M - y_K) - (y_N - y_M)*(x_M - x_K)) / denominator;
    float t_J = ((x_L - x_K)*(y_M - y_K) - (y_L - y_K)*(x_M - x_K)) / denominator;

    if (s_J >= 0 && s_J <= 1 && t_J >= 0 && t_J <= 1) {
        intersection.x = x_K + (x_L - x_K) * s_J;
        intersection.y = y_K + (y_L - y_K) * s_J;
        return true;
    }

    return false;
}

void Graph2D::processIntersections() {
    std::vector<Line2D> newEdges;
    std::vector<Point2D> newVertices = vertices;
    std::unordered_map<size_t, size_t> indexMap; // Old index -> New index

    for (size_t i = 0; i < edges.size(); ++i) {
        Line2D& line1 = edges[i];
        const Point2D& A1 = vertices[line1.startIdx];
        const Point2D& A2 = vertices[line1.endIdx];

        for (size_t j = i + 1; j < edges.size(); ++j) {
            Line2D& line2 = edges[j];
            const Point2D& B1 = vertices[line2.startIdx];
            const Point2D& B2 = vertices[line2.endIdx];

            Point2D intersectionPoint;
            if (findIntersection(A1, A2, B1, B2, intersectionPoint)) {
                // Add intersection point to vertices
                auto it = std::find(newVertices.begin(), newVertices.end(), intersectionPoint);
                size_t intersectionIdx;
                if (it == newVertices.end()) {
                    intersectionIdx = newVertices.size();
                    newVertices.push_back(intersectionPoint);
                    std::cout << "[Debug][" << viewName << "] Added intersection point at (" << intersectionPoint.x << ", " << intersectionPoint.y << ")" << std::endl;
                } else {
                    intersectionIdx = std::distance(newVertices.begin(), it);
                }

                // Split line1 into two segments
                Line2D newLine1(line1.startIdx, intersectionIdx, line1.type);
                Line2D newLine2(intersectionIdx, line1.endIdx, line1.type);
                newEdges.push_back(newLine1);
                newEdges.push_back(newLine2);

                // Remove the original line1
                line1.startIdx = -1; // Mark as invalid

                // Split line2 into two segments
                Line2D newLine3(line2.startIdx, intersectionIdx, line2.type);
                Line2D newLine4(intersectionIdx, line2.endIdx, line2.type);
                newEdges.push_back(newLine3);
                newEdges.push_back(newLine4);

                // Remove the original line2
                line2.startIdx = -1; // Mark as invalid
            }
        }
    }

    // Remove invalid edges
    edges.erase(std::remove_if(edges.begin(), edges.end(),
        [](const Line2D& edge) { return edge.startIdx == -1; }), edges.end());

    // Add new edges
    edges.insert(edges.end(), newEdges.begin(), newEdges.end());

    // Update vertices
    vertices = newVertices;
}

void Graph2D::handleCollinearLines() {
    // For simplicity, this implementation does not merge collinear lines
    // Implementing collinear line merging is beyond the scope of this example
    std::cout << "[Debug][" << viewName << "] Collinear line handling not implemented." << std::endl;
}

void Graph2D::print() const {
    std::cout << viewName << " View:" << std::endl;
    std::cout << "Vertices:" << std::endl;
    for (size_t i = 0; i < vertices.size(); ++i) {
        std::cout << "Vertex " << i << ": ";
        vertices[i].print();
    }
    std::cout << "Edges:" << std::endl;
    for (const auto& e : edges) {
        printf("Edge from Vertex %d to Vertex %d [%s]\n", e.startIdx, e.endIdx, e.type.c_str());
    }
}