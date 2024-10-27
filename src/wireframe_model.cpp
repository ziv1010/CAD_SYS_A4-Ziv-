#include "wireframe_model.h"
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fstream>

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

void WireframeModel::writeVerticesToFile(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "[Error] Cannot open output file " << filename << std::endl;
        return;
    }

    outfile << "Probable 3D Vertices:" << std::endl;
    for (const auto& v : probableVertices) {
        outfile << v.x << " " << v.y << " " << v.z << std::endl;
    }

    outfile.close();
    std::cout << "[Debug] Probable 3D vertices written to " << filename << std::endl;
}

void WireframeModel::print() const {
    std::cout << "Probable 3D Vertices:" << std::endl;
    for (const auto& v : probableVertices) {
        v.print();
    }
}