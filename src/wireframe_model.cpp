#include "wireframe_model.h"
#include <iostream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#define EPSILON 1e-6

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

            bool coincideInThirdView = false;
            if (lineCount == 2) {
                // Check if projections coincide in the third view
                if (!edgeInTopView) {
                    coincideInThirdView = (p1_top == p2_top);
                } else if (!edgeInFrontView) {
                    coincideInThirdView = (p1_front == p2_front);
                } else if (!edgeInSideView) {
                    coincideInThirdView = (p1_side == p2_side);
                }
            }

            // Only add edge if conditions are met
            if (lineCount == 3 || (lineCount == 2 && coincideInThirdView)) {
                // Before adding the edge, check for containment (existing logic)
                // ... existing edge containment code ...

                // Add new edge if not contained
                probableEdges.push_back(Edge3D(i, j));
                std::cout << "[Debug] Added Edge3D from Vertex " << i << " to Vertex " << j << std::endl;
            } else {
                std::cout << "[Debug] Edge between Vertex " << i << " and Vertex " << j << " not added. Conditions not met." << std::endl;
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

    outfile << "Faces:" << std::endl;
    for (const auto& face : probableFaces) {
        outfile << "Face with vertices:";
        for (int idx : face.vertexIndices) {
            outfile << " " << idx;
        }
        outfile << std::endl;
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

    std::cout << "Probable 3D Faces:" << std::endl;
    for (const auto& face : probableFaces) {
        face.print();
    }
}


bool WireframeModel::isPointOnLineSegment(const Vertex3D& p, const Vertex3D& a, const Vertex3D& b) const {
    // Check if point p lies on line segment ab
    // First, check if p lies on the line defined by a and b
    Vertex3D ap(p.x - a.x, p.y - a.y, p.z - a.z);
    Vertex3D ab(b.x - a.x, b.y - a.y, b.z - a.z);
    float cross_x = ap.y * ab.z - ap.z * ab.y;
    float cross_y = ap.z * ab.x - ap.x * ab.z;
    float cross_z = ap.x * ab.y - ap.y * ab.x;
    float cross_norm = sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);

    if (cross_norm > EPSILON) {
        // Not colinear
        return false;
    }

    // Check if p is between a and b
    float dotProduct = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y) + (p.z - a.z) * (b.z - a.z);
    if (dotProduct < 0)
        return false;

    float squaredLengthAB = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y) + (b.z - a.z)*(b.z - a.z);
    if (dotProduct > squaredLengthAB)
        return false;

    return true;
}

bool WireframeModel::isEdgeContained(const Edge3D& e1, const Edge3D& e2) const {
    // Check if edge e2 is contained within edge e1
    const Vertex3D& a = probableVertices[e1.startIdx];
    const Vertex3D& b = probableVertices[e1.endIdx];
    const Vertex3D& c = probableVertices[e2.startIdx];
    const Vertex3D& d = probableVertices[e2.endIdx];

    return isPointOnLineSegment(c, a, b) && isPointOnLineSegment(d, a, b);
}

// Helper method to get edges adjacent to a vertex
std::vector<int> WireframeModel::getAdjacentEdges(int vertexIdx) const {
    std::vector<int> adjacentEdges;
    for (size_t i = 0; i < probableEdges.size(); ++i) {
        const Edge3D& edge = probableEdges[i];
        if (edge.startIdx == vertexIdx || edge.endIdx == vertexIdx) {
            adjacentEdges.push_back(i);
        }
    }
    return adjacentEdges;
}

// Helper method to compute internal angle between two edges at a vertex
float WireframeModel::computeInternalAngle(const Vertex3D& v, const Vertex3D& v1, const Vertex3D& v2) const {
    // Vectors from v to v1 and v to v2
    Vertex3D vec1(v1.x - v.x, v1.y - v.y, v1.z - v.z);
    Vertex3D vec2(v2.x - v.x, v2.y - v.y, v2.z - v.z);

    // Compute dot product and magnitudes
    float dotProd = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    float mag1 = sqrt(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
    float mag2 = sqrt(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);

    if (mag1 < EPSILON || mag2 < EPSILON)
        return 0.0f;

    float cosTheta = dotProd / (mag1 * mag2);

    // Clamp cosTheta to [-1,1] to avoid numerical issues
    cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

    // Return the angle in radians
    return acos(cosTheta);
}

// Helper method to get vector representation of an edge
Vertex3D WireframeModel::edgeVector(int startIdx, int endIdx) const {
    const Vertex3D& startVertex = probableVertices[startIdx];
    const Vertex3D& endVertex = probableVertices[endIdx];
    return Vertex3D(endVertex.x - startVertex.x, endVertex.y - startVertex.y, endVertex.z - startVertex.z);
}

// Helper method to compute cross product of two vectors
Vertex3D WireframeModel::crossProduct(const Vertex3D& a, const Vertex3D& b) const {
    return Vertex3D(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

void WireframeModel::generateProbableFaces() {
    std::cout << "[Debug] Generating probable faces..." << std::endl;

    // Set to store unique faces (represented by sorted list of vertex indices)
    std::set<std::vector<int>> uniqueFaces;

    // Build adjacency list for edges
    std::map<int, std::vector<int>> vertexToEdges;
    for (size_t i = 0; i < probableEdges.size(); ++i) {
        const Edge3D& edge = probableEdges[i];
        vertexToEdges[edge.startIdx].push_back(i);
        vertexToEdges[edge.endIdx].push_back(i);
    }

    // For each vertex
    for (size_t vi = 0; vi < probableVertices.size(); ++vi) {
        int v_i = vi;
        const Vertex3D& vertex_i = probableVertices[v_i];

        // Get adjacent edges
        std::vector<int> adjacentEdges = vertexToEdges[v_i];

        // For each pair of adjacent edges at vertex v_i
        for (size_t e1_idx = 0; e1_idx < adjacentEdges.size(); ++e1_idx) {
            for (size_t e2_idx = e1_idx + 1; e2_idx < adjacentEdges.size(); ++e2_idx) {
                int edgeIdx1 = adjacentEdges[e1_idx];
                int edgeIdx2 = adjacentEdges[e2_idx];

                // Initialize face trace
                std::vector<int> faceVertices;
                faceVertices.push_back(v_i);

                int currentVertex = v_i;
                int previousEdgeIdx = edgeIdx1;
                int currentEdgeIdx = edgeIdx2;

                std::vector<int> faceEdges;
                faceEdges.push_back(previousEdgeIdx);
                faceEdges.push_back(currentEdgeIdx);

                std::vector<Vertex3D> normals;

                bool faceCompleted = false;
                std::set<std::pair<int, int>> visitedEdges;
                visitedEdges.insert({previousEdgeIdx, currentEdgeIdx});

                while (true) {
                    // Get the other vertex of currentEdge
                    const Edge3D& currentEdge = probableEdges[currentEdgeIdx];
                    int nextVertex = (currentEdge.startIdx == currentVertex) ? currentEdge.endIdx : currentEdge.startIdx;

                    if (nextVertex == v_i) {
                        // Face completed
                        faceCompleted = true;
                        break;
                    }

                    faceVertices.push_back(nextVertex);

                    // Get adjacent edges of nextVertex, excluding currentEdge
                    std::vector<int> nextAdjacentEdges = vertexToEdges[nextVertex];
                    // Remove currentEdge
                    nextAdjacentEdges.erase(std::remove(nextAdjacentEdges.begin(), nextAdjacentEdges.end(), currentEdgeIdx), nextAdjacentEdges.end());

                    if (nextAdjacentEdges.empty()) {
                        // No further edges, cannot continue
                        break;
                    }

                    // Compute internal angles between currentEdge and each adjacent edge
                    int selectedEdgeIdx = -1;
                    float minAngle = std::numeric_limits<float>::max();

                    const Vertex3D& currentVertexPos = probableVertices[currentVertex];
                    const Vertex3D& nextVertexPos = probableVertices[nextVertex];

                    Vertex3D vec1 = edgeVector(currentVertex, nextVertex);

                    for (int adjEdgeIdx : nextAdjacentEdges) {
                        const Edge3D& adjEdge = probableEdges[adjEdgeIdx];

                        int otherVertex = (adjEdge.startIdx == nextVertex) ? adjEdge.endIdx : adjEdge.startIdx;
                        const Vertex3D& otherVertexPos = probableVertices[otherVertex];

                        Vertex3D vec2 = edgeVector(nextVertex, otherVertex);

                        // Compute internal angle
                        float angle = computeInternalAngle(nextVertexPos, currentVertexPos, otherVertexPos);

                        // Planarity check using cross products
                        Vertex3D normal1 = crossProduct(vec1, vec2);
                        if (normals.empty()) {
                            normals.push_back(normal1);
                        } else {
                            // Compare with initial normal
                            Vertex3D& normal0 = normals[0];
                            float dotProduct = normal0.x * normal1.x + normal0.y * normal1.y + normal0.z * normal1.z;
                            float mag0 = sqrt(normal0.x * normal0.x + normal0.y * normal0.y + normal0.z * normal0.z);
                            float mag1 = sqrt(normal1.x * normal1.x + normal1.y * normal1.y + normal1.z * normal1.z);

                            if (mag0 < EPSILON || mag1 < EPSILON) {
                                continue; // Degenerate normal
                            }

                            float cosTheta = dotProduct / (mag0 * mag1);
                            cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

                            if (fabs(cosTheta - 1.0f) > EPSILON) {
                                continue; // Not planar
                            }
                        }

                        if (angle < minAngle) {
                            minAngle = angle;
                            selectedEdgeIdx = adjEdgeIdx;
                        }
                    }

                    if (selectedEdgeIdx == -1) {
                        // No suitable edge found
                        break;
                    }

                    // Avoid cycles
                    std::pair<int, int> edgePair = {currentEdgeIdx, selectedEdgeIdx};
                    if (visitedEdges.count(edgePair)) {
                        break;
                    }
                    visitedEdges.insert(edgePair);

                    // Update for next iteration
                    faceEdges.push_back(selectedEdgeIdx);
                    previousEdgeIdx = currentEdgeIdx;
                    currentEdgeIdx = selectedEdgeIdx;
                    currentVertex = nextVertex;
                    vec1 = edgeVector(currentVertex, probableEdges[currentEdgeIdx].startIdx == currentVertex ? probableEdges[currentEdgeIdx].endIdx : probableEdges[currentEdgeIdx].startIdx);
                }

                if (faceCompleted) {
                    // Create face
                    // Sort vertex indices to avoid duplicates
                    std::vector<int> faceVertexIndices = faceVertices;
                    std::sort(faceVertexIndices.begin(), faceVertexIndices.end());

                    if (uniqueFaces.find(faceVertexIndices) == uniqueFaces.end()) {
                        uniqueFaces.insert(faceVertexIndices);
                        probableFaces.push_back(Face3D(faceVertices));
                        std::cout << "[Debug] Found probable face with vertices:";
                        for (int idx : faceVertices) {
                            std::cout << " " << idx;
                        }
                        std::cout << std::endl;
                    }
                }
            }
        }
    }

    std::cout << "[Debug] Total probable faces found: " << probableFaces.size() << std::endl;
}
