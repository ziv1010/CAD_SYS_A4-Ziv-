#include "wireframe_model.h"
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fstream>
#include <map>
#include <set>


void WireframeModel::generateProbableVertices(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView) {
    // Maps to find matching coordinates
    std::unordered_multimap<float, int> topXMap;     // x -> indices in top view
    std::unordered_multimap<float, int> topZMap;     // z -> indices in top view (y in 2D)

    std::unordered_multimap<float, int> frontXMap;   // x -> indices in front view
    std::unordered_multimap<float, int> frontYMap;   // y -> indices in front view

    std::unordered_multimap<float, int> sideYMap;    // y -> indices in side view (x in 2D)
    std::unordered_multimap<float, int> sideZMap;    // z -> indices in side view (y in 2D)

    // Populate maps
    for (size_t i = 0; i < topView.vertices.size(); ++i) {
        const auto& v = topView.vertices[i];
        topXMap.emplace(v.x, i);
        topZMap.emplace(v.y, i); // In top view, y corresponds to z
    }

    for (size_t i = 0; i < frontView.vertices.size(); ++i) {
        const auto& v = frontView.vertices[i];
        frontXMap.emplace(v.x, i);
        frontYMap.emplace(v.y, i);
    }

    for (size_t i = 0; i < sideView.vertices.size(); ++i) {
        const auto& v = sideView.vertices[i];
        sideYMap.emplace(v.x, i); // In side view, x corresponds to y
        sideZMap.emplace(v.y, i); // In side view, y corresponds to z
    }

    // Generate probable 3D vertices
    for (const auto& [x, frontIndices] : frontXMap) {
        // For each x in front view, find matching x in top view
        auto rangeTopX = topXMap.equal_range(x);
        for (auto itTopX = rangeTopX.first; itTopX != rangeTopX.second; ++itTopX) {
            int topIdx = itTopX->second;
            float z = topView.vertices[topIdx].y; // y in top view corresponds to z

            // For the current x, get all y in front view
            float y_front = frontView.vertices[frontIndices].y;

            // Find matching y in side view (x in side view)
            auto rangeSideY = sideYMap.equal_range(y_front);
            for (auto itSideY = rangeSideY.first; itSideY != rangeSideY.second; ++itSideY) {
                int sideIdx = itSideY->second;
                float z_side = sideView.vertices[sideIdx].y; // y in side view corresponds to z

                // Check if z from top view matches z from side view
                if (std::fabs(z - z_side) < 1e-6) {
                    // Probable 3D vertex found
                    Vertex3D vertex3D(x, y_front, z);
                    probableVertices.push_back(vertex3D);
                    std::cout << "[Debug] Probable Vertex3D(" << x << ", " << y_front << ", " << z << ")" << std::endl;
                }
            }
        }
    }
}

void WireframeModel::generateProbableEdges(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView) {
    std::cout << "[Debug] Generating probable 3D edges..." << std::endl;

    // For each pair of probable vertices
    for (size_t i = 0; i < probableVertices.size(); ++i) {
        for (size_t j = i + 1; j < probableVertices.size(); ++j) {
            const Vertex3D& v1 = probableVertices[i];
            const Vertex3D& v2 = probableVertices[j];

            // Check if their projections correspond to edges in the 2D views
            bool edgeInTopView = false;
            bool edgeInFrontView = false;
            bool edgeInSideView = false;

            // Check Top View
            Point2D p1_top(v1.x, v1.z); // Top view projects onto x-z plane
            Point2D p2_top(v2.x, v2.z);
            for (const auto& edge : topView.edges) {
                const Point2D& e1 = topView.vertices[edge.startIdx];
                const Point2D& e2 = topView.vertices[edge.endIdx];
                if ((p1_top == e1 && p2_top == e2) || (p1_top == e2 && p2_top == e1)) {
                    edgeInTopView = true;
                    break;
                }
            }

            // Check Front View
            Point2D p1_front(v1.x, v1.y); // Front view projects onto x-y plane
            Point2D p2_front(v2.x, v2.y);
            for (const auto& edge : frontView.edges) {
                const Point2D& e1 = frontView.vertices[edge.startIdx];
                const Point2D& e2 = frontView.vertices[edge.endIdx];
                if ((p1_front == e1 && p2_front == e2) || (p1_front == e2 && p2_front == e1)) {
                    edgeInFrontView = true;
                    break;
                }
            }

            // Check Side View
            Point2D p1_side(v1.y, v1.z); // Side view projects onto y-z plane
            Point2D p2_side(v2.y, v2.z);
            for (const auto& edge : sideView.edges) {
                const Point2D& e1 = sideView.vertices[edge.startIdx];
                const Point2D& e2 = sideView.vertices[edge.endIdx];
                if ((p1_side == e1 && p2_side == e2) || (p1_side == e2 && p2_side == e1)) {
                    edgeInSideView = true;
                    break;
                }
            }

            // Decide if the edge should be added
            int lineCount = edgeInTopView + edgeInFrontView + edgeInSideView;
            if (lineCount >= 2) {
                // Edge appears in at least two views
                Edge3D edge(i, j);
                // Check for redundancy
                auto it = std::find(probableEdges.begin(), probableEdges.end(), edge);
                if (it == probableEdges.end()) {
                    probableEdges.push_back(edge);
                    std::cout << "[Debug] Added Edge3D from Vertex " << i << " to Vertex " << j << std::endl;
                }
            }
        }
    }
}

void WireframeModel::validateVerticesAndEdges() {
    std::cout << "[Debug] Validating vertices and edges..." << std::endl;
    bool changesMade;
    do {
        changesMade = false;
        // Build adjacency list
        std::map<int, std::set<int>> adjacencyList;
        for (const auto& edge : probableEdges) {
            adjacencyList[edge.startIdx].insert(edge.endIdx);
            adjacencyList[edge.endIdx].insert(edge.startIdx);
        }

        // Find invalid vertices
        std::set<int> invalidVertices;
        for (size_t i = 0; i < probableVertices.size(); ++i) {
            int connections = adjacencyList[i].size();
            if (connections < 3) {
                invalidVertices.insert(i);
                std::cout << "[Debug] Vertex " << i << " is invalid (connections: " << connections << ")" << std::endl;
            }
        }

        if (!invalidVertices.empty()) {
            changesMade = true;
            // Remove invalid vertices and associated edges
            std::vector<Vertex3D> newVertices;
            std::vector<int> indexMap(probableVertices.size(), -1);
            int newIndex = 0;
            for (size_t i = 0; i < probableVertices.size(); ++i) {
                if (invalidVertices.find(i) == invalidVertices.end()) {
                    newVertices.push_back(probableVertices[i]);
                    indexMap[i] = newIndex++;
                } else {
                    std::cout << "[Debug] Removing Vertex " << i << std::endl;
                }
            }

            probableVertices = newVertices;

            // Update edges
            std::vector<Edge3D> newEdges;
            for (const auto& edge : probableEdges) {
                if (invalidVertices.find(edge.startIdx) == invalidVertices.end() &&
                    invalidVertices.find(edge.endIdx) == invalidVertices.end()) {
                    Edge3D newEdge(indexMap[edge.startIdx], indexMap[edge.endIdx]);
                    newEdges.push_back(newEdge);
                } else {
                    std::cout << "[Debug] Removing Edge from Vertex " << edge.startIdx << " to Vertex " << edge.endIdx << std::endl;
                }
            }

            probableEdges = newEdges;
        }

    } while (changesMade);

    std::cout << "[Debug] Validation complete." << std::endl;
}

void WireframeModel::writeToFile(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[Error] Cannot open output file " << filename << std::endl;
        return;
    }

    outfile << "Vertices:" << std::endl;
    for (size_t i = 0; i < probableVertices.size(); ++i) {
        const auto& v = probableVertices[i];
        outfile << i << ": " << v.x << " " << v.y << " " << v.z << std::endl;
    }

    outfile << "Edges:" << std::endl;
    for (const auto& edge : probableEdges) {
        outfile << edge.startIdx << " " << edge.endIdx << std::endl;
    }

    outfile.close();
    std::cout << "[Debug] Wireframe model written to " << filename << std::endl;
}

void WireframeModel::print() const {
    std::cout << "Probable 3D Vertices:" << std::endl;
    for (size_t i = 0; i < probableVertices.size(); ++i) {
        std::cout << "Vertex " << i << ": ";
        probableVertices[i].print();
    }

    std::cout << "Probable 3D Edges:" << std::endl;
    for (const auto& edge : probableEdges) {
        edge.print();
    }
}