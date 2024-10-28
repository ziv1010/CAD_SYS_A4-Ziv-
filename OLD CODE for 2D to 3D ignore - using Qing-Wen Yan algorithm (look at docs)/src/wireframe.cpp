// wireframe.cpp
#include "wireframe.h"
#include "plane.h"
#include "vector3d.h"
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "vector2d.h"
#include <stack>
#include <algorithm>
#include <cmath>
#include "FaceLoopSide.h"

// Constructor
Wireframe::Wireframe(const Projection2D& front, const Projection2D& top, const Projection2D& side)
    : frontView(front), topView(top), sideView(side) {
    generate3DVertices();
    generate3DEdges();
    removeRedundantEdges();
    removePathologicalElements();
}

// Helper function to find if a 3D vertex already exists
int Wireframe::findVertex3DIndex(float x, float y, float z) const {
    for (size_t i = 0; i < vertices3D.size(); ++i) {
        if (std::abs(vertices3D[i].getX() - x) < 1e-6 &&
            std::abs(vertices3D[i].getY() - y) < 1e-6 &&
            std::abs(vertices3D[i].getZ() - z) < 1e-6) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

// Generate 3D vertices by matching 2D vertices from projections
void Wireframe::generate3DVertices() {
    // Mapping from projection vertices to 3D vertices
    std::unordered_map<int, int> frontTo3D, topTo3D, sideTo3D;

    // For simplicity, assume that the projections are aligned and matching by indices
    // In practice, more sophisticated matching is required
    size_t numVertices = std::min({ frontView.vertices.size(), topView.vertices.size(), sideView.vertices.size() });

    for (size_t i = 0; i < numVertices; ++i) {
        const Vertex2D& vf = frontView.vertices[i];
        const Vertex2D& vt = topView.vertices[i];
        const Vertex2D& vs = sideView.vertices[i];

        float x = vf.getX();
        float y = vt.getY(); // From top view (Y axis)
        float z = vs.getY(); // From side view (Z axis)

        // Check if vertex already exists
        int existingIndex = findVertex3DIndex(x, y, z);
        if (existingIndex == -1) {
            vertices3D.emplace_back(x, y, z);
            existingIndex = static_cast<int>(vertices3D.size() - 1);
        }

        frontTo3D[i] = existingIndex;
        topTo3D[i] = existingIndex;
        sideTo3D[i] = existingIndex;
    }
}

// Generate 3D edges by combining 2D edges from projections
void Wireframe::generate3DEdges() {
    // A simple approach: for each edge in front view, check if corresponding edges exist in top and side views
    for (const auto& fe : frontView.edges) {
        int fv1_idx = fe.v1_index;
        int fv2_idx = fe.v2_index;

        // Get corresponding 3D vertex indices
        if (fv1_idx >= vertices3D.size() || fv2_idx >= vertices3D.size()) continue;

        int v1_idx = fv1_idx;
        int v2_idx = fv2_idx;

        // Check if this edge exists in top and side views
        bool edgeInTop = false;
        bool edgeInSide = false;

        for (const auto& te : topView.edges) {
            if ((te.v1_index == fv1_idx && te.v2_index == fv2_idx) ||
                (te.v1_index == fv2_idx && te.v2_index == fv1_idx)) {
                edgeInTop = true;
                break;
            }
        }

        for (const auto& se : sideView.edges) {
            if ((se.v1_index == fv1_idx && se.v2_index == fv2_idx) ||
                (se.v1_index == fv2_idx && se.v2_index == fv1_idx)) {
                edgeInSide = true;
                break;
            }
        }

        if (edgeInTop && edgeInSide) {
            // Add edge if it exists in all projections
            edges3D.emplace_back(v1_idx, v2_idx);
            // Update connectivity
            vertices3D[v1_idx].addConnectedEdge(static_cast<int>(edges3D.size() - 1));
            vertices3D[v2_idx].addConnectedEdge(static_cast<int>(edges3D.size() - 1));
        }
    }

    // Repeat for edges in top and side views to ensure all edges are captured
    // This can be enhanced based on specific requirements
}

// Remove redundant edges using the RER procedure
void Wireframe::removeRedundantEdges() {
    // Mark all edges as unexamined
    std::vector<bool> examined(edges3D.size(), false);

    for (size_t i = 0; i < edges3D.size(); ++i) {
        if (examined[i]) continue;

        const Edge3D& e1 = edges3D[i];
        examined[i] = true;

        std::vector<size_t> overlappingEdges;

        for (size_t j = i + 1; j < edges3D.size(); ++j) {
            if (examined[j]) continue;

            const Edge3D& e2 = edges3D[j];

            // Check if edges overlap (same vertices, regardless of direction)
            if ((e1.v1_index == e2.v1_index && e1.v2_index == e2.v2_index) ||
                (e1.v1_index == e2.v2_index && e1.v2_index == e2.v1_index)) {
                overlappingEdges.push_back(j);
            }
        }

        // Remove overlapping edges
        // Iterate in reverse to avoid invalidating indices
        for (auto it = overlappingEdges.rbegin(); it != overlappingEdges.rend(); ++it) {
            int idx = static_cast<int>(*it);
            // Remove edge from connected vertices
            vertices3D[edges3D[idx].v1_index].removeConnectedEdge(idx);
            vertices3D[edges3D[idx].v2_index].removeConnectedEdge(idx);
            // Remove edge from edges3D
            edges3D.erase(edges3D.begin() + idx);
            // Update connectivity indices in all vertices
            for (auto& vertex : vertices3D) {
                for (auto& edgeIdx : vertex.connectedEdges) {
                    if (edgeIdx > idx) {
                        edgeIdx--;
                    }
                }
            }
            // Adjust examined vector
            examined.erase(examined.begin() + idx);
        }
    }
}

// Helper function to check if two edges are collinear
bool Wireframe::areCollinear(int edge1, int edge2) const {
    if (edge1 >= edges3D.size() || edge2 >= edges3D.size()) return false;

    const Vertex3D& v1_start = vertices3D[edges3D[edge1].v1_index];
    const Vertex3D& v1_end = vertices3D[edges3D[edge1].v2_index];
    const Vertex3D& v2_start = vertices3D[edges3D[edge2].v1_index];
    const Vertex3D& v2_end = vertices3D[edges3D[edge2].v2_index];

    // Compute direction vectors
    float dx1 = v1_end.x - v1_start.x;
    float dy1 = v1_end.y - v1_start.y;
    float dz1 = v1_end.z - v1_start.z;

    float dx2 = v2_end.x - v2_start.x;
    float dy2 = v2_end.y - v2_start.y;
    float dz2 = v2_end.z - v2_start.z;

    // Compute cross product
    float cross_x = dy1 * dz2 - dz1 * dy2;
    float cross_y = dz1 * dx2 - dx1 * dz2;
    float cross_z = dx1 * dy2 - dy1 * dx2;

    // If cross product is (close to) zero vector, they are collinear
    return (std::abs(cross_x) < 1e-6 &&
            std::abs(cross_y) < 1e-6 &&
            std::abs(cross_z) < 1e-6);
}

// Helper function to check if a set of edges are coplanar
bool Wireframe::areCoplanar(const std::vector<int>& edgeIndices) const {
    if (edgeIndices.size() < 3) return true; // Any two edges are always coplanar

    // Get three distinct points to define a plane
    const Vertex3D& p1 = vertices3D[edges3D[edgeIndices[0]].v1_index];
    const Vertex3D& p2 = vertices3D[edges3D[edgeIndices[0]].v2_index];
    const Vertex3D& p3 = vertices3D[edges3D[edgeIndices[1]].v2_index];

    // Define vectors
    float v1x = p2.x - p1.x;
    float v1y = p2.y - p1.y;
    float v1z = p2.z - p1.z;

    float v2x = p3.x - p1.x;
    float v2y = p3.y - p1.y;
    float v2z = p3.z - p1.z; // Corrected from p3.z - p3.z

    // Compute normal vector of the plane
    float nx = v1y * v2z - v1z * v2y;
    float ny = v1z * v2x - v1x * v2z;
    float nz = v1x * v2y - v1y * v2x;

    // Normalize the normal vector
    float length = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (length < 1e-6) return false; // Degenerate case, cannot define plane

    nx /= length;
    ny /= length;
    nz /= length;

    // Compute d using p1
    float d = nx * p1.x + ny * p1.y + nz * p1.z;

    // Check if all other edges lie on this plane
    for (size_t i = 0; i < edgeIndices.size(); ++i) {
        const Edge3D& edge = edges3D[edgeIndices[i]];
        const Vertex3D& va = vertices3D[edge.v1_index];
        const Vertex3D& vb = vertices3D[edge.v2_index];

        float dist_a = nx * va.x + ny * va.y + nz * va.z - d;
        float dist_b = nx * vb.x + ny * vb.y + nz * vb.z - d;

        if (std::abs(dist_a) > 1e-6 || std::abs(dist_b) > 1e-6) {
            return false;
        }
    }

    return true;
}

// Helper function to merge two collinear edges into one
void Wireframe::mergeEdges(int edge1, int edge2) {
    if (edge1 >= edges3D.size() || edge2 >= edges3D.size()) return;

    const Edge3D& e1 = edges3D[edge1];
    const Edge3D& e2 = edges3D[edge2];

    // Determine the common vertex
    int common = -1;
    if (e1.v1_index == e2.v1_index || e1.v1_index == e2.v2_index) {
        common = e1.v1_index;
    }
    else if (e1.v2_index == e2.v1_index || e1.v2_index == e2.v2_index) {
        common = e1.v2_index;
    }

    if (common == -1) return; // No common vertex, cannot merge

    // Determine the non-common vertices
    int non_common_e1 = (e1.v1_index == common) ? e1.v2_index : e1.v1_index;
    int non_common_e2 = (e2.v1_index == common) ? e2.v2_index : e2.v1_index;

    // Create a new edge connecting the two non-common vertices
    Edge3D newEdge(non_common_e1, non_common_e2); // Declaration
    edges3D.emplace_back(newEdge);
    int newEdgeIdx = static_cast<int>(edges3D.size() - 1);

    // Update connectivity
    vertices3D[non_common_e1].addConnectedEdge(newEdgeIdx);
    vertices3D[non_common_e2].addConnectedEdge(newEdgeIdx);

    // Remove the old edges from connected vertices
    vertices3D[e1.v1_index].removeConnectedEdge(edge1);
    vertices3D[e1.v2_index].removeConnectedEdge(edge1);
    vertices3D[e2.v1_index].removeConnectedEdge(edge2);
    vertices3D[e2.v2_index].removeConnectedEdge(edge2);

    // Remove the old edges from edges3D
    // Remove higher index first to avoid invalidating the lower index
    if (edge1 > edge2) {
        edges3D.erase(edges3D.begin() + edge1);
        edges3D.erase(edges3D.begin() + edge2);
    }
    else {
        edges3D.erase(edges3D.begin() + edge2);
        edges3D.erase(edges3D.begin() + edge1);
    }

    // Update connectivity indices in all vertices
    for (auto& vertex : vertices3D) {
        for (auto& eIdx : vertex.connectedEdges) {
            if (eIdx > edge1) eIdx--;
            if (eIdx > edge2) eIdx--;
            // New edge is already added at the end, no need to adjust
        }
    }
}

// Remove pathological edges and vertices using the PEVR procedure
void Wireframe::removePathologicalElements() {
    bool changesMade = true;

    while (changesMade) {
        changesMade = false;

        for (size_t i = 0; i < vertices3D.size(); ++i) {
            Vertex3D& vertex = vertices3D[i];
            int degree = vertex.degree();

            if (degree == 0) {
                // W3.1: Delete isolated vertex
                // Remove vertex and continue
                vertices3D.erase(vertices3D.begin() + i);
                // No edges connected, so no need to update edges
                // Update edge indices in all vertices
                for (auto& v : vertices3D) {
                    for (auto& edgeIdx : v.connectedEdges) {
                        if (edgeIdx > static_cast<int>(i)) {
                            edgeIdx--;
                        }
                    }
                }
                changesMade = true;
                i--; // Adjust index after erase
                continue;
            }
            else if (degree == 1) {
                // W3.2: Remove dangling edge and vertex
                int edgeIdx = vertex.connectedEdges[0];
                Edge3D& edge = edges3D[edgeIdx];
                // Find the other vertex
                int otherV = (edge.v1_index == static_cast<int>(i)) ? edge.v2_index : edge.v1_index;
                // Remove the edge from the other vertex's connectedEdges
                vertices3D[otherV].removeConnectedEdge(edgeIdx);
                // Remove the edge from edges3D
                edges3D.erase(edges3D.begin() + edgeIdx);
                // Remove the edge from current vertex's connectedEdges
                vertex.removeConnectedEdge(edgeIdx);
                // Remove vertex
                vertices3D.erase(vertices3D.begin() + i);
                // Update edge indices in all vertices
                for (auto& v : vertices3D) {
                    for (auto& eIdx : v.connectedEdges) {
                        if (eIdx > edgeIdx) {
                            eIdx--;
                        }
                    }
                }
                changesMade = true;
                i--; // Adjust index after erase
                continue;
            }
            else if (degree == 2) {
                // W3.3: Handle degree 2 vertices
                int edge1 = vertex.connectedEdges[0];
                int edge2 = vertex.connectedEdges[1];

                if (areCollinear(edge1, edge2)) {
                    // Merge the two collinear edges
                    mergeEdges(edge1, edge2);
                    changesMade = true;
                    i--; // Recheck the current index after modification
                    break;
                }
                else {
                    // Remove both edges and the vertex
                    // Remove edges from connected vertices
                    Edge3D& e1 = edges3D[edge1];
                    Edge3D& e2 = edges3D[edge2];

                    int otherV1 = (e1.v1_index == static_cast<int>(i)) ? e1.v2_index : e1.v1_index;
                    int otherV2 = (e2.v1_index == static_cast<int>(i)) ? e2.v2_index : e2.v1_index;

                    vertices3D[otherV1].removeConnectedEdge(edge1);
                    vertices3D[otherV2].removeConnectedEdge(edge2);

                    // Remove edges from edges3D
                    // Remove higher index first
                    if (edge1 > edge2) {
                        edges3D.erase(edges3D.begin() + edge1);
                        edges3D.erase(edges3D.begin() + edge2);
                    }
                    else {
                        edges3D.erase(edges3D.begin() + edge2);
                        edges3D.erase(edges3D.begin() + edge1);
                    }

                    // Remove vertex
                    vertices3D.erase(vertices3D.begin() + i);
                    // Update edge indices in all vertices
                    for (auto& v : vertices3D) {
                        for (auto& eIdx : v.connectedEdges) {
                            if (eIdx > edge1) eIdx--;
                            if (eIdx > edge2) eIdx--;
                        }
                    }
                    changesMade = true;
                    i--; // Adjust index after erase
                    break;
                }
            }
            else if (degree == 3) {
                // W3.4 and W3.5: Handle degree 3 vertices
                std::vector<int> connectedEdges = vertex.connectedEdges;
                bool twoCollinear = false;
                int collinearEdge1 = -1, collinearEdge2 = -1;
                for (size_t m = 0; m < connectedEdges.size(); ++m) {
                    for (size_t n = m + 1; n < connectedEdges.size(); ++n) {
                        if (areCollinear(connectedEdges[m], connectedEdges[n])) {
                            twoCollinear = true;
                            collinearEdge1 = connectedEdges[m];
                            collinearEdge2 = connectedEdges[n];
                            break;
                        }
                    }
                    if (twoCollinear) break;
                }

                if (twoCollinear) {
                    // W3.4: Two edges are collinear
                    // Merge the two collinear edges and remove the third edge
                    int thirdEdge = -1;
                    for (const auto& eIdx : connectedEdges) {
                        if (eIdx != collinearEdge1 && eIdx != collinearEdge2) {
                            thirdEdge = eIdx;
                            break;
                        }
                    }
                    if (thirdEdge != -1) {
                        // Remove the third edge
                        Edge3D& te = edges3D[thirdEdge];
                        int otherV = (te.v1_index == static_cast<int>(i)) ? te.v2_index : te.v1_index;
                        vertices3D[otherV].removeConnectedEdge(thirdEdge);
                        // Remove the edge from edges3D
                        edges3D.erase(edges3D.begin() + thirdEdge);
                        // Remove the edge from current vertex's connectedEdges
                        vertex.removeConnectedEdge(thirdEdge);
                        // Merge collinear edges
                        mergeEdges(collinearEdge1, collinearEdge2);
                        changesMade = true;
                        i--; // Recheck after modification
                        break;
                    }
                }
                else {
                    // Check coplanarity
                    bool coplanar = areCoplanar(connectedEdges);
                    if (coplanar) {
                        // W3.5: Three edges are coplanar but no two are collinear
                        // Remove the vertex and all three edges
                        for (const auto& eIdx : connectedEdges) {
                            Edge3D& e = edges3D[eIdx];
                            int otherV = (e.v1_index == static_cast<int>(i)) ? e.v2_index : e.v1_index;
                            vertices3D[otherV].removeConnectedEdge(eIdx);
                        }
                        // Remove edges in reverse order to maintain correct indices
                        std::sort(connectedEdges.begin(), connectedEdges.end(), std::greater<int>());
                        for (const auto& eIdx : connectedEdges) {
                            edges3D.erase(edges3D.begin() + eIdx);
                        }
                        // Remove vertex
                        vertices3D.erase(vertices3D.begin() + i);
                        // Update edge indices in all vertices
                        for (auto& v : vertices3D) {
                            for (auto& eIdx : v.connectedEdges) {
                                for (const auto& removedIdx : connectedEdges) {
                                    if (eIdx > removedIdx) {
                                        eIdx--;
                                    }
                                }
                            }
                        }
                        changesMade = true;
                        i--; // Adjust index after erase
                        break;
                    }
                }
            }
            else if (degree >= 4) {
                // W3.6: Handle degree >=4 vertices
                // For simplicity, handle only the first four edges
                std::vector<int> firstFourEdges;
                for (int eIdx : vertex.connectedEdges) {
                    firstFourEdges.push_back(eIdx);
                    if (firstFourEdges.size() == 4) break;
                }

                if (firstFourEdges.size() < 4) continue;

                bool coplanar = areCoplanar(firstFourEdges);
                if (coplanar) {
                    // Count number of collinear pairs
                    int collinearPairs = 0;
                    for (size_t m = 0; m < firstFourEdges.size(); ++m) {
                        for (size_t n = m + 1; n < firstFourEdges.size(); ++n) {
                            if (areCollinear(firstFourEdges[m], firstFourEdges[n])) {
                                collinearPairs++;
                            }
                        }
                    }

                    if (collinearPairs == 1) {
                        // Only two edges are collinear
                        // Merge the two collinear edges and remove two non-collinear edges
                        int collinearEdge1 = -1, collinearEdge2 = -1;
                        for (size_t m = 0; m < firstFourEdges.size(); ++m) {
                            for (size_t n = m + 1; n < firstFourEdges.size(); ++n) {
                                if (areCollinear(firstFourEdges[m], firstFourEdges[n])) {
                                    collinearEdge1 = firstFourEdges[m];
                                    collinearEdge2 = firstFourEdges[n];
                                    break;
                                }
                            }
                            if (collinearEdge1 != -1) break;
                        }

                        if (collinearEdge1 != -1 && collinearEdge2 != -1) {
                            // Identify two non-collinear edges
                            std::vector<int> nonCollinearEdges;
                            for (const auto& eIdx : firstFourEdges) {
                                if (eIdx != collinearEdge1 && eIdx != collinearEdge2) {
                                    nonCollinearEdges.push_back(eIdx);
                                }
                            }

                            if (nonCollinearEdges.size() >= 2) {
                                int nonCollinearEdge1 = nonCollinearEdges[0];
                                int nonCollinearEdge2 = nonCollinearEdges[1];

                                // Merge collinear edges
                                mergeEdges(collinearEdge1, collinearEdge2);

                                // Remove non-collinear edges
                                Edge3D& e1 = edges3D[nonCollinearEdge1];
                                Edge3D& e2 = edges3D[nonCollinearEdge2];
                                int otherV1 = (e1.v1_index == static_cast<int>(i)) ? e1.v2_index : e1.v1_index;
                                int otherV2 = (e2.v1_index == static_cast<int>(i)) ? e2.v2_index : e2.v1_index;

                                vertices3D[otherV1].removeConnectedEdge(nonCollinearEdge1);
                                vertices3D[otherV2].removeConnectedEdge(nonCollinearEdge2);

                                // Remove edges from edges3D
                                // Remove higher index first
                                if (nonCollinearEdge1 > nonCollinearEdge2) {
                                    edges3D.erase(edges3D.begin() + nonCollinearEdge1);
                                    edges3D.erase(edges3D.begin() + nonCollinearEdge2);
                                }
                                else {
                                    edges3D.erase(edges3D.begin() + nonCollinearEdge2);
                                    edges3D.erase(edges3D.begin() + nonCollinearEdge1);
                                }

                                // Remove vertex
                                vertices3D.erase(vertices3D.begin() + i);

                                // Update edge indices in all vertices
                                for (auto& v : vertices3D) {
                                    for (auto& eIdx : v.connectedEdges) {
                                        if (eIdx > nonCollinearEdge1) eIdx--;
                                        if (eIdx > nonCollinearEdge2) eIdx--;
                                    }
                                }

                                changesMade = true;
                                i--; // Adjust index after erase
                                break;
                            }
                        }
                    }
                    else if (collinearPairs >= 2) {
                        // Two pairs of collinear edges or all edges are non-collinear
                        // Remove vertex and all connected edges
                        for (const auto& eIdx : vertex.connectedEdges) {
                            Edge3D& e = edges3D[eIdx];
                            int otherV = (e.v1_index == static_cast<int>(i)) ? e.v2_index : e.v1_index;
                            vertices3D[otherV].removeConnectedEdge(eIdx);
                        }
                        // Remove edges in reverse order to maintain correct indices
                        std::vector<int> connectedEdgesCopy = vertex.connectedEdges;
                        std::sort(connectedEdgesCopy.begin(), connectedEdgesCopy.end(), std::greater<int>());
                        for (const auto& eIdx : connectedEdgesCopy) {
                            edges3D.erase(edges3D.begin() + eIdx);
                        }
                        // Remove vertex
                        vertices3D.erase(vertices3D.begin() + i);
                        // Update edge indices in all vertices
                        for (auto& v : vertices3D) {
                            for (auto& eIdx : v.connectedEdges) {
                                for (const auto& removedIdx : connectedEdgesCopy) {
                                    if (eIdx > removedIdx) {
                                        eIdx--;
                                    }
                                }
                            }
                        }
                        changesMade = true;
                        i--; // Adjust index after erase
                        break;
                    }
                }
            }
        }
    }
}
// Save wireframe to file
// Definition in wireframe.cpp
void Wireframe::saveToFile(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to write to file " << filename << std::endl;
        return;
    }

    // Output vertices
    int numVertices = static_cast<int>(vertices3D.size());
    outfile << numVertices << std::endl;
    for (const auto& vertex : vertices3D) {
        outfile << vertex.getX() << " " << vertex.getY() << " " << vertex.getZ() << std::endl;
    }

    // Output edges
    int numEdges = static_cast<int>(edges3D.size());
    outfile << numEdges << std::endl;
    for (const auto& edge : edges3D) {
        outfile << edge.v1_index << " " << edge.v2_index << std::endl;
    }

    outfile.close();
}

void Wireframe::generatePlanarGraphs() {
    // P1: Record the adjacent edges for each vertex
    // This is already handled via Vertex3D.connectedEdges

    // P2: Construct planes
    std::vector<Plane> tempPlanes;
    for (size_t vi = 0; vi < vertices3D.size(); ++vi) {
        const Vertex3D& vertex = vertices3D[vi];
        const std::vector<int>& connectedEdges = vertex.connectedEdges;

        // For each pair of non-collinear adjacent edges
        for (size_t m = 0; m < connectedEdges.size(); ++m) {
            for (size_t n = m + 1; n < connectedEdges.size(); ++n) {
                int edgeIdx1 = connectedEdges[m];
                int edgeIdx2 = connectedEdges[n];

                const Edge3D& edge1 = edges3D[edgeIdx1];
                const Edge3D& edge2 = edges3D[edgeIdx2];

                // Get the other vertices of the edges
                int vj = (edge1.v1_index == vi) ? edge1.v2_index : edge1.v1_index;
                int vk = (edge2.v1_index == vi) ? edge2.v2_index : edge2.v1_index;

                const Vertex3D& vertex_j = vertices3D[vj];
                const Vertex3D& vertex_k = vertices3D[vk];

                // Form two vectors
                Vector3D vec1(vertex_j.x - vertex.x, vertex_j.y - vertex.y, vertex_j.z - vertex.z);
                Vector3D vec2(vertex_k.x - vertex.x, vertex_k.y - vertex.y, vertex_k.z - vertex.z);

                // Check for collinearity
                Vector3D cross = vec1.cross(vec2);
                if (cross.length() < 1e-6) {
                    // Vectors are collinear, skip
                    continue;
                }

                // P3: Hypothesize the outer normal vector of the plane
                Vector3D normal = cross.normalize();

                // Plane equation: a(x - x0) + b(y - y0) + c(z - z0) = 0
                // Simplify to: ax + by + cz + d = 0
                float a = normal.x;
                float b = normal.y;
                float c = normal.z;
                float d = -(a * vertex.x + b * vertex.y + c * vertex.z);

                // Adjust normal direction based on d
                if (d < 0) {
                    a = -a;
                    b = -b;
                    c = -c;
                    d = -d;
                }

                // Create a new plane
                Plane newPlane(a, b, c, d);

                // P4: Eliminate duplicated planes
                bool isDuplicate = false;
                for (const auto& existingPlane : tempPlanes) {
                    if (newPlane.isSimilar(existingPlane)) {
                        isDuplicate = true;
                        break;
                    }
                }
                if (!isDuplicate) {
                    tempPlanes.push_back(newPlane);
                }
            }
        }
    }

    // P5: Search for all edges on the plane
    for (auto& plane : tempPlanes) {
        for (size_t ei = 0; ei < edges3D.size(); ++ei) {
            const Edge3D& edge = edges3D[ei];
            const Vertex3D& v1 = vertices3D[edge.v1_index];
            const Vertex3D& v2 = vertices3D[edge.v2_index];

            // Compute distances to plane
            float D1 = plane.a * v1.x + plane.b * v1.y + plane.c * v1.z + plane.d;
            float D2 = plane.a * v2.x + plane.b * v2.y + plane.c * v2.z + plane.d;

            if (std::abs(D1) <= 1e-5f && std::abs(D2) <= 1e-5f) {
                plane.edgesOnPlane.push_back(static_cast<int>(ei));
            }
        }
    }

    // P6: Check the legality of each planar graph
planes.clear();
for (auto& plane : tempPlanes) {
    // P6.1: If a planar graph consists of two or fewer edges, discard
    if (plane.edgesOnPlane.size() <= 2) {
        continue;
    }

    // P6.2: If an edge belongs to only one plane, remove it
    std::vector<int> validEdges;
    for (int edgeIdx : plane.edgesOnPlane) {
        int planeCount = 0;
        for (const auto& otherPlane : tempPlanes) {
            if (&plane == &otherPlane) continue;
            if (std::find(otherPlane.edgesOnPlane.begin(), otherPlane.edgesOnPlane.end(), edgeIdx) != otherPlane.edgesOnPlane.end()) {
                planeCount++;
                break; // No need to count further if found in another plane
            }
        }
        if (planeCount > 0) {
            validEdges.push_back(edgeIdx);
        } else {
            // Edge belongs to only one plane, remove it
            // Apply PEVR procedure if necessary
            int v1 = edges3D[edgeIdx].v1_index;
            int v2 = edges3D[edgeIdx].v2_index;

            // Mark edge as invalid (you may need to add this member variable)
            edges3D[edgeIdx].isValid = false;

            // Remove the edge from connected vertices
            vertices3D[v1].removeConnectedEdge(edgeIdx);
            vertices3D[v2].removeConnectedEdge(edgeIdx);

            // Check if vertices have become pathological (degree 0 or 1)
            if (vertices3D[v1].degree() <= 1) {
                removePathologicalElementsAtVertex(v1);
            }
            if (vertices3D[v2].degree() <= 1) {
                removePathologicalElementsAtVertex(v2);
            }
        }
    }

    plane.edgesOnPlane = validEdges;

    // P6.1 (Recheck): If after removing edges, the planar graph has two or fewer edges, discard
    if (plane.edgesOnPlane.size() <= 2) {
        continue;
    }

    // P6.3: Check if the boundary is closed
    if (isPlanarGraphClosed(plane)) {
        planes.push_back(plane);
    } else {
        // Remove unclosed planar graphs
        // Optionally, remove dangling edges and recheck
        removeDanglingEdgesFromPlane(plane);
        // Recheck if the boundary is closed
        if (isPlanarGraphClosed(plane)) {
            planes.push_back(plane);
        }
        // If still unclosed, discard the plane
    }
}
}

bool Wireframe::isPlanarGraphClosed(const Plane& plane) const {
    // Build a map of vertex degrees within the plane
    std::unordered_map<int, std::vector<int>> vertexEdges;
    for (int edgeIdx : plane.edgesOnPlane) {
        if (edgeIdx >= edges3D.size()) continue;
        const Edge3D& edge = edges3D[edgeIdx];
        if (!edge.isValid) continue;

        vertexEdges[edge.v1_index].push_back(edge.v2_index);
        vertexEdges[edge.v2_index].push_back(edge.v1_index);
    }

    // For a closed loop, the graph must be connected, and each vertex must have degree 2
    // Check connectivity using BFS
    if (vertexEdges.empty()) return false;

    std::unordered_set<int> visited;
    std::queue<int> q;
    int startVertex = vertexEdges.begin()->first;
    q.push(startVertex);
    visited.insert(startVertex);

    while (!q.empty()) {
        int current = q.front();
        q.pop();
        for (int neighbor : vertexEdges[current]) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                q.push(neighbor);
            }
        }
    }

    // Check if all vertices are visited
    if (visited.size() != vertexEdges.size()) return false;

    // Check if each vertex has degree 2
    for (const auto& ve : vertexEdges) {
        if (ve.second.size() != 2) {
            return false;
        }
    }

    return true;
}


void Wireframe::removePathologicalElementsAtVertex(int vertexIndex) {
    if (vertexIndex >= vertices3D.size()) return;

    Vertex3D& vertex = vertices3D[vertexIndex];
    int degree = vertex.degree();

    if (degree == 0) {
        // Remove isolated vertex
        vertices3D.erase(vertices3D.begin() + vertexIndex);
        // Adjust indices in edges and other vertices
        adjustIndicesAfterVertexRemoval(vertexIndex);
    } else if (degree == 1) {
        // Remove dangling edge and vertex
        int edgeIdx = vertex.connectedEdges[0];
        if (edgeIdx >= edges3D.size()) return;

        Edge3D& edge = edges3D[edgeIdx];
        int otherV = (edge.v1_index == vertexIndex) ? edge.v2_index : edge.v1_index;

        // Remove edge from other vertex
        vertices3D[otherV].removeConnectedEdge(edgeIdx);

        // Mark edge as invalid
        edges3D[edgeIdx].isValid = false;

        // Remove vertex
        vertices3D.erase(vertices3D.begin() + vertexIndex);

        // Adjust indices in edges and other vertices
        adjustIndicesAfterVertexRemoval(vertexIndex);
        adjustEdgeIndicesAfterEdgeRemoval(edgeIdx);
    }
}

void Wireframe::adjustIndicesAfterVertexRemoval(int removedVertexIndex) {
    // Adjust vertex indices in edges
    for (auto& edge : edges3D) {
        if (edge.v1_index > removedVertexIndex) edge.v1_index--;
        if (edge.v2_index > removedVertexIndex) edge.v2_index--;
    }
    // Adjust connected edges in vertices
    for (auto& vertex : vertices3D) {
        for (auto& edgeIdx : vertex.connectedEdges) {
            // No change needed here
        }
    }
}

void Wireframe::adjustEdgeIndicesAfterEdgeRemoval(int removedEdgeIndex) {
    // Adjust edge indices in vertices
    for (auto& vertex : vertices3D) {
        for (auto& edgeIdx : vertex.connectedEdges) {
            if (edgeIdx > removedEdgeIndex) edgeIdx--;
        }
    }
}

void Wireframe::removeDanglingEdgesFromPlane(Plane& plane) {
    bool edgesRemoved = true;
    while (edgesRemoved) {
        edgesRemoved = false;
        std::unordered_map<int, int> vertexDegree;

        // Calculate vertex degrees
        for (int edgeIdx : plane.edgesOnPlane) {
            if (edgeIdx >= edges3D.size()) continue;
            const Edge3D& edge = edges3D[edgeIdx];
            if (!edge.isValid) continue;

            vertexDegree[edge.v1_index]++;
            vertexDegree[edge.v2_index]++;
        }

        std::vector<int> edgesToRemove;

        // Identify dangling edges
        for (int edgeIdx : plane.edgesOnPlane) {
            if (edgeIdx >= edges3D.size()) continue;
            const Edge3D& edge = edges3D[edgeIdx];
            if (!edge.isValid) continue;

            int v1 = edge.v1_index;
            int v2 = edge.v2_index;

            if (vertexDegree[v1] == 1 || vertexDegree[v2] == 1) {
                edgesToRemove.push_back(edgeIdx);
                edges3D[edgeIdx].isValid = false;
                vertices3D[v1].removeConnectedEdge(edgeIdx);
                vertices3D[v2].removeConnectedEdge(edgeIdx);
                edgesRemoved = true;
            }
        }

        // Remove edges from plane
        for (int edgeIdx : edgesToRemove) {
            plane.edgesOnPlane.erase(
                std::remove(plane.edgesOnPlane.begin(), plane.edgesOnPlane.end(), edgeIdx),
                plane.edgesOnPlane.end());
        }
    }
}


// In wireframe.cpp

void Wireframe::generateFaceLoops() {
    for (auto& plane : planes) {
        // F1: Construct vertex-edge adjacency table for the plane
        std::unordered_map<int, std::vector<int>> adjTab; // Vertex index to list of edge indices

        for (int edgeIdx : plane.edgesOnPlane) {
            const Edge3D& edge = edges3D[edgeIdx];
            if (!edge.isValid) continue;

            adjTab[edge.v1_index].push_back(edgeIdx);
            adjTab[edge.v2_index].push_back(edgeIdx);
        }

        // F2: Find the ordered adjacent edges of each vertex
        std::unordered_map<int, std::vector<int>> orderedEdgesAtVertex; // Vertex index to ordered edge indices

        for (const auto& entry : adjTab) {
            int vertexIdx = entry.first;
            const std::vector<int>& connectedEdges = entry.second;

            // F2.1: Select an entry (v, n_e, SE) from adjTab
            // F2.2: Order the adjacent edges based on the clockwise angle around the plane normal
            std::vector<int> orderedEdges = orderEdgesAtVertex(vertexIdx, connectedEdges, plane);

            orderedEdgesAtVertex[vertexIdx] = orderedEdges;
        }

        // F3: Find all basic loops within the planar graph
        std::vector<bool> edgeVisited(edges3D.size(), false);

        for (int edgeIdx : plane.edgesOnPlane) {
            if (edgeVisited[edgeIdx]) continue;

            std::vector<int> basicLoop;

            // Start traversing from this edge in both directions
            if (findBasicLoop(edgeIdx, plane, orderedEdgesAtVertex, edgeVisited, basicLoop)) {
                plane.basicLoops.push_back(basicLoop);
            }
        }

        // F4: Identify inclusion relationships between basic loops
        std::vector<std::vector<bool>> inclusionMatrix = computeInclusionMatrix(plane);

        // F5: Form face loops
        plane.faceLoops = formFaceLoops(plane.basicLoops, inclusionMatrix);
    }
}


std::vector<int> Wireframe::orderEdgesAtVertex(int vertexIdx, const std::vector<int>& connectedEdges, const Plane& plane) {
    std::vector<std::pair<float, int>> angleEdgePairs;

    const Vertex3D& vertex = vertices3D[vertexIdx];

    // Reference vector (arbitrary). We'll use the first edge's vector as reference.
    Vector3D refVector;
    if (!connectedEdges.empty()) {
        const Edge3D& refEdge = edges3D[connectedEdges[0]];
        int otherVertexIdx = (refEdge.v1_index == vertexIdx) ? refEdge.v2_index : refEdge.v1_index;
        const Vertex3D& otherVertex = vertices3D[otherVertexIdx];
        refVector = Vector3D(otherVertex.x - vertex.x, otherVertex.y - vertex.y, otherVertex.z - vertex.z).normalize();
    } else {
        return {}; // No connected edges
    }

    // Plane normal vector
    Vector3D normal(plane.a, plane.b, plane.c);

    for (int edgeIdx : connectedEdges) {
        const Edge3D& edge = edges3D[edgeIdx];
        int otherVertexIdx = (edge.v1_index == vertexIdx) ? edge.v2_index : edge.v1_index;
        const Vertex3D& otherVertex = vertices3D[otherVertexIdx];

        Vector3D edgeVector(otherVertex.x - vertex.x, otherVertex.y - vertex.y, otherVertex.z - vertex.z);
        edgeVector = edgeVector.normalize();

        // Calculate angle with respect to refVector
        float angle = std::atan2(normal.cross(refVector).dot(edgeVector), refVector.dot(edgeVector));

        angleEdgePairs.emplace_back(angle, edgeIdx);
    }

    // Sort edges by angle
    std::sort(angleEdgePairs.begin(), angleEdgePairs.end(),
              [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                  return a.first < b.first;
              });

    // Extract ordered edge indices
    std::vector<int> orderedEdges;
    for (const auto& pair : angleEdgePairs) {
        orderedEdges.push_back(pair.second);
    }

    return orderedEdges;
}

bool Wireframe::findBasicLoop(int startEdgeIdx, const Plane& plane,
                              const std::unordered_map<int, std::vector<int>>& orderedEdgesAtVertex,
                              std::vector<bool>& edgeVisited, std::vector<int>& basicLoop) {
    std::stack<std::pair<int, int>> edgeStack; // Pair of edge index and current vertex index
    std::unordered_set<int> visitedEdgesInLoop;

    const Edge3D& startEdge = edges3D[startEdgeIdx];
    int startVertexIdx = startEdge.v1_index;
    int currentVertexIdx = startEdge.v2_index;

    edgeStack.push({startEdgeIdx, currentVertexIdx});
    basicLoop.push_back(startEdgeIdx);
    visitedEdgesInLoop.insert(startEdgeIdx);
    edgeVisited[startEdgeIdx] = true;

    while (!edgeStack.empty()) {
        int edgeIdx = edgeStack.top().first;
        currentVertexIdx = edgeStack.top().second;
        edgeStack.pop();

        const auto& orderedEdges = orderedEdgesAtVertex.at(currentVertexIdx);

        // Find the next edge in orderedEdges that hasn't been visited yet
        bool foundNextEdge = false;
        for (int nextEdgeIdx : orderedEdges) {
            if (visitedEdgesInLoop.find(nextEdgeIdx) != visitedEdgesInLoop.end())
                continue; // Already in current loop

            const Edge3D& nextEdge = edges3D[nextEdgeIdx];
            int nextVertexIdx = (nextEdge.v1_index == currentVertexIdx) ? nextEdge.v2_index : nextEdge.v1_index;

            // Add to loop
            basicLoop.push_back(nextEdgeIdx);
            visitedEdgesInLoop.insert(nextEdgeIdx);
            edgeVisited[nextEdgeIdx] = true;

            if (nextVertexIdx == startVertexIdx) {
                // Loop closed
                return true;
            }

            edgeStack.push({nextEdgeIdx, nextVertexIdx});
            foundNextEdge = true;
            break;
        }

        if (!foundNextEdge) {
            // Dead end, backtrack
            if (!basicLoop.empty()) {
                int backEdgeIdx = basicLoop.back();
                basicLoop.pop_back();
                visitedEdgesInLoop.erase(backEdgeIdx);
            } else {
                return false;
            }
        }
    }

    return false;
}
std::vector<std::vector<bool>> Wireframe::computeInclusionMatrix(const Plane& plane) {
    size_t numLoops = plane.basicLoops.size();
    std::vector<std::vector<bool>> inclusionMatrix(numLoops, std::vector<bool>(numLoops, false));

    // For each pair of loops, determine inclusion
    for (size_t i = 0; i < numLoops; ++i) {
        for (size_t j = 0; j < numLoops; ++j) {
            if (i == j) continue;

            if (isLoopIncludedInLoop(plane.basicLoops[i], plane.basicLoops[j], plane)) {
                inclusionMatrix[i][j] = true; // Loop i is included in Loop j
            }
        }
    }

    return inclusionMatrix;
}

std::vector<std::vector<int>> Wireframe::formFaceLoops(const std::vector<std::vector<int>>& basicLoops,
                                                       const std::vector<std::vector<bool>>& inclusionMatrix) {
    std::vector<std::vector<int>> faceLoops;

    size_t numLoops = basicLoops.size();
    std::vector<bool> loopProcessed(numLoops, false);

    for (size_t i = 0; i < numLoops; ++i) {
        if (loopProcessed[i]) continue;

        // Check if loop i includes other loops
        std::vector<int> includedLoops;
        for (size_t j = 0; j < numLoops; ++j) {
            if (inclusionMatrix[j][i]) {
                includedLoops.push_back(j);
                loopProcessed[j] = true;
            }
        }

        // Form face loop
        std::vector<int> faceLoop = basicLoops[i];
        for (int includedLoopIdx : includedLoops) {
            // Inner loops can be stored separately if needed
            // For now, we can append them or store as nested loops
            // For simplicity, we can keep faceLoop as is
        }

        faceLoops.push_back(faceLoop);
        loopProcessed[i] = true;
    }

    return faceLoops;
}

bool Wireframe::isLoopIncludedInLoop(const std::vector<int>& innerLoop, const std::vector<int>& outerLoop, const Plane& plane) {
    // For simplicity, we'll check if a point from innerLoop is inside outerLoop
    // This can be done using point-in-polygon test in 2D projection

    // Project vertices onto the plane
    std::vector<Vector2D> outerLoop2D;
    std::vector<Vector2D> innerLoop2D;

    projectLoopOntoPlane(outerLoop, plane, outerLoop2D);
    projectLoopOntoPlane(innerLoop, plane, innerLoop2D);

    // Take a point from innerLoop and check if it's inside outerLoop
    if (innerLoop2D.empty() || outerLoop2D.empty()) return false;

    Vector2D testPoint = innerLoop2D[0];
    return isPointInPolygon(testPoint, outerLoop2D);
}

void Wireframe::projectLoopOntoPlane(const std::vector<int>& loop, const Plane& plane, std::vector<Vector2D>& loop2D) {
    // Define a coordinate system on the plane
    Vector3D normal(plane.a, plane.b, plane.c);
    Vector3D refVector;

    // Find two vectors on the plane to define 2D axes
    if (std::abs(normal.x) > std::abs(normal.y)) {
        refVector = Vector3D(-normal.z, 0, normal.x).normalize();
    } else {
        refVector = Vector3D(0, -normal.z, normal.y).normalize();
    }
    Vector3D refVector2 = normal.cross(refVector).normalize();

    // Project each vertex onto the plane coordinate system
    for (int edgeIdx : loop) {
        const Edge3D& edge = edges3D[edgeIdx];

        int vIdx1 = edge.v1_index;
        int vIdx2 = edge.v2_index;

        const Vertex3D& v1 = vertices3D[vIdx1];
        const Vertex3D& v2 = vertices3D[vIdx2];

        Vector3D point(v1.x, v1.y, v1.z);
        float x = point.dot(refVector);
        float y = point.dot(refVector2);
        loop2D.emplace_back(x, y);

        // Assuming loop is defined by vertices, we can skip v2 if it's the same as v1
    }
}

bool Wireframe::isPointInPolygon(const Vector2D& point, const std::vector<Vector2D>& polygon) {
    int crossings = 0;
    size_t count = polygon.size();
    for (size_t i = 0; i < count; ++i) {
        const Vector2D& a = polygon[i];
        const Vector2D& b = polygon[(i + 1) % count];

        if (((a.y > point.y) != (b.y > point.y)) &&
            (point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y + 1e-6f) + a.x)) {
            crossings++;
        }
    }
    return (crossings % 2) == 1;
}

// In wireframe.cpp

void Wireframe::handleCuttingEdgesAndVertices() {
    // For each pair of planes
    for (size_t i = 0; i < planes.size(); ++i) {
        for (size_t j = i + 1; j < planes.size(); ++j) {
            Plane& plane1 = planes[i];
            Plane& plane2 = planes[j];

            // Check if planes are coplanar
            if (arePlanesCoplanar(plane1, plane2)) {
                continue; // Skip coplanar planes
            }

            // Compute the intersection line of the two planes
            Vector3D pointOnLine, lineDirection;
            if (!computePlaneIntersectionLine(plane1, plane2, pointOnLine, lineDirection)) {
                continue; // Planes are parallel and non-intersecting
            }

            // For each pair of face loops
            for (const auto& faceLoop1 : plane1.faceLoops) {
                for (const auto& faceLoop2 : plane2.faceLoops) {
                    int caseType = 0;
                    if (checkFaceLoopsIntersection(plane1, plane2, faceLoop1, faceLoop2, caseType)) {
                        if (caseType == 1) {
                            // Case 1: Regular intersection, no action needed
                            continue;
                        } else if (caseType == 2) {
                            // Case 2: Introduce cutting edges and vertices
                            std::vector<Vector3D> intersectionPoints;
                            if (findFaceLoopIntersections(faceLoop1, faceLoop2, intersectionPoints)) {
                                insertCuttingEdgesAndVertices(plane1, plane2, intersectionPoints);
                            }
                        } else if (caseType == 3) {
                            // Case 3: Remove the problematic face loop
                            removeFaceLoop(plane1, faceLoop1);
                            // Optionally, break out of loops if necessary
                        }
                    }
                }
            }
        }
    }
}

bool Wireframe::arePlanesCoplanar(const Plane& plane1, const Plane& plane2, float tolerance) const {
    // Compute the cross product of the normals
    Vector3D normal1(plane1.a, plane1.b, plane1.c);
    Vector3D normal2(plane2.a, plane2.b, plane2.c);
    Vector3D cross = normal1.cross(normal2);

    if (cross.length() <= tolerance) {
        // Normals are parallel; check if planes are the same
        float dDiff = std::abs(plane1.d - plane2.d);
        return dDiff <= tolerance;
    }
    return false;
}

bool Wireframe::computePlaneIntersectionLine(const Plane& plane1, const Plane& plane2, Vector3D& pointOnLine, Vector3D& lineDirection) {
    Vector3D n1(plane1.a, plane1.b, plane1.c);
    Vector3D n2(plane2.a, plane2.b, plane2.c);

    lineDirection = n1.cross(n2);

    float denom = lineDirection.length();

    if (denom < 1e-6f) {
        // Planes are parallel and non-intersecting
        return false;
    }

    // Solve for a point on the line of intersection
    // Using Cramer's Rule for solving linear equations

    // Plane equations:
    // plane1: a1*x + b1*y + c1*z + d1 = 0
    // plane2: a2*x + b2*y + c2*z + d2 = 0

    // We can set one variable (e.g., x) to 0 and solve for y and z

    // Choose the largest component of lineDirection to avoid division by zero
    float absX = std::abs(lineDirection.x);
    float absY = std::abs(lineDirection.y);
    float absZ = std::abs(lineDirection.z);

    float x, y, z;

    if (absX >= absY && absX >= absZ) {
        // Set y = 0, z = 0
        x = -(plane1.d * plane2.b - plane2.d * plane1.b) / (plane1.a * plane2.b - plane2.a * plane1.b);
        y = 0;
        z = 0;
    } else if (absY >= absX && absY >= absZ) {
        // Set x = 0, z = 0
        x = 0;
        y = -(plane1.d * plane2.c - plane2.d * plane1.c) / (plane1.b * plane2.c - plane2.b * plane1.c);
        z = 0;
    } else {
        // Set x = 0, y = 0
        x = 0;
        y = 0;
        z = -(plane1.d * plane2.a - plane2.d * plane1.a) / (plane1.c * plane2.a - plane2.c * plane1.a);
    }

    pointOnLine = Vector3D(x, y, z);

    lineDirection = lineDirection.normalize();

    return true;
}

bool Wireframe::checkFaceLoopsIntersection(const Plane& plane1, const Plane& plane2, const std::vector<int>& faceLoop1, const std::vector<int>& faceLoop2, int& caseType) {
    // Project face loops onto the intersection line and check for overlap

    Vector3D pointOnLine, lineDirection;
    if (!computePlaneIntersectionLine(plane1, plane2, pointOnLine, lineDirection)) {
        return false;
    }

    // Collect vertices of faceLoop1
    std::vector<Vector3D> faceLoop1Vertices;
    for (int edgeIdx : faceLoop1) {
        const Edge3D& edge = edges3D[edgeIdx];
        const Vertex3D& v1 = vertices3D[edge.v1_index];
        const Vertex3D& v2 = vertices3D[edge.v2_index];
        faceLoop1Vertices.push_back(Vector3D(v1.x, v1.y, v1.z));
        faceLoop1Vertices.push_back(Vector3D(v2.x, v2.y, v2.z));
    }

    // Collect vertices of faceLoop2
    std::vector<Vector3D> faceLoop2Vertices;
    for (int edgeIdx : faceLoop2) {
        const Edge3D& edge = edges3D[edgeIdx];
        const Vertex3D& v1 = vertices3D[edge.v1_index];
        const Vertex3D& v2 = vertices3D[edge.v2_index];
        faceLoop2Vertices.push_back(Vector3D(v1.x, v1.y, v1.z));
        faceLoop2Vertices.push_back(Vector3D(v2.x, v2.y, v2.z));
    }

    // Check if the intersection line intersects both face loops
    bool intersectsFaceLoop1 = faceLoopIntersectsLine(faceLoop1Vertices, pointOnLine, lineDirection);
    bool intersectsFaceLoop2 = faceLoopIntersectsLine(faceLoop2Vertices, pointOnLine, lineDirection);

    if (intersectsFaceLoop1 && intersectsFaceLoop2) {
        // Now classify the intersection into cases

        // For simplicity, we'll assume:
        // - If the intersection line coincides with an edge in both face loops, it's Case 1.
        // - If the intersection line crosses both face loops at edges or vertices, it's Case 2.
        // - If the intersection line is entirely inside one face loop without touching boundaries, it's Case 3.

        bool coincidesWithEdge = lineCoincidesWithEdge(faceLoop1Vertices, pointOnLine, lineDirection) &&
                                 lineCoincidesWithEdge(faceLoop2Vertices, pointOnLine, lineDirection);

        if (coincidesWithEdge) {
            caseType = 1;
            return true;
        }

        bool touchesBoundaries = lineTouchesFaceLoopBoundaries(faceLoop1Vertices, pointOnLine, lineDirection) &&
                                 lineTouchesFaceLoopBoundaries(faceLoop2Vertices, pointOnLine, lineDirection);

        if (touchesBoundaries) {
            caseType = 2;
            return true;
        }

        caseType = 3;
        return true;
    }

    return false; // No intersection
}

bool Wireframe::faceLoopIntersectsLine(const std::vector<Vector3D>& faceLoopVertices, const Vector3D& pointOnLine, const Vector3D& lineDirection) {
    // For each edge in the face loop, check if it intersects with the line
    for (size_t i = 0; i < faceLoopVertices.size(); i += 2) {
        const Vector3D& p1 = faceLoopVertices[i];
        const Vector3D& p2 = faceLoopVertices[i + 1];

        if (lineIntersectsSegment(pointOnLine, lineDirection, p1, p2)) {
            return true;
        }
    }
    return false;
}

bool Wireframe::lineIntersectsSegment(const Vector3D& linePoint, const Vector3D& lineDir, const Vector3D& segPoint1, const Vector3D& segPoint2) {
    // Compute the closest point between line and segment
    Vector3D u = lineDir;
    Vector3D v = segPoint2 - segPoint1;
    Vector3D w = linePoint - segPoint1;

    float a = u.dot(u); // always >= 0
    float b = u.dot(v);
    float c = v.dot(v); // always >= 0
    float d = u.dot(w);
    float e = v.dot(w);

    float D = a * c - b * b; // always >= 0

    float sc, tc;

    // Compute the line parameters of the two closest points
    if (D < 1e-6f) {
        // Lines are almost parallel
        sc = 0.0f;
        tc = (b > c ? d / b : e / c);
    } else {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }

    // Check if tc is between 0 and 1 (segment parameter)
    if (tc < 0.0f || tc > 1.0f) {
        return false;
    }

    // Compute the closest points
    Vector3D closestPointLine = linePoint + u * sc;
    Vector3D closestPointSegment = segPoint1 + v * tc;

    // Compute the distance between the closest points
    Vector3D diff = closestPointLine - closestPointSegment;
    float distanceSquared = diff.dot(diff);

    // If the distance is small enough, consider it an intersection
    return distanceSquared < 1e-6f;
}

bool Wireframe::lineCoincidesWithEdge(const std::vector<Vector3D>& faceLoopVertices, const Vector3D& pointOnLine, const Vector3D& lineDirection) {
    // For each edge in the face loop, check if the line coincides with the edge
    for (size_t i = 0; i < faceLoopVertices.size(); i += 2) {
        const Vector3D& p1 = faceLoopVertices[i];
        const Vector3D& p2 = faceLoopVertices[i + 1];

        Vector3D edgeDir = p2 - p1;
        edgeDir = edgeDir.normalize();

        // Check if edge direction and line direction are the same or opposite
        float dotProduct = std::abs(edgeDir.dot(lineDirection));
        if (dotProduct > 1.0f - 1e-6f) {
            // Check if the line passes through p1
            Vector3D diff = p1 - pointOnLine;
            Vector3D crossProduct = diff.cross(lineDirection);
            if (crossProduct.length() < 1e-6f) {
                return true;
            }
        }
    }
    return false;
}

bool Wireframe::lineTouchesFaceLoopBoundaries(const std::vector<Vector3D>& faceLoopVertices, const Vector3D& pointOnLine, const Vector3D& lineDirection) {
    // Check if the line intersects at the edges or vertices of the face loop
    for (size_t i = 0; i < faceLoopVertices.size(); i += 2) {
        const Vector3D& p1 = faceLoopVertices[i];
        const Vector3D& p2 = faceLoopVertices[i + 1];

        // Check intersection with edge
        if (lineIntersectsSegment(pointOnLine, lineDirection, p1, p2)) {
            return true;
        }

        // Check if line passes through vertex p1 or p2
        Vector3D diff1 = p1 - pointOnLine;
        Vector3D cross1 = diff1.cross(lineDirection);
        if (cross1.length() < 1e-6f) {
            return true;
        }

        Vector3D diff2 = p2 - pointOnLine;
        Vector3D cross2 = diff2.cross(lineDirection);
        if (cross2.length() < 1e-6f) {
            return true;
        }
    }
    return false;
}

bool Wireframe::findFaceLoopIntersections(const std::vector<int>& faceLoop1, const std::vector<int>& faceLoop2, std::vector<Vector3D>& intersectionPoints) {
    // Implement intersection point calculation between faceLoop1 and faceLoop2
    // For each edge in faceLoop1, check for intersection with each edge in faceLoop2
    for (int edgeIdx1 : faceLoop1) {
        const Edge3D& edge1 = edges3D[edgeIdx1];
        Vector3D p1_start(vertices3D[edge1.v1_index].x, vertices3D[edge1.v1_index].y, vertices3D[edge1.v1_index].z);
        Vector3D p1_end(vertices3D[edge1.v2_index].x, vertices3D[edge1.v2_index].y, vertices3D[edge1.v2_index].z);

        for (int edgeIdx2 : faceLoop2) {
            const Edge3D& edge2 = edges3D[edgeIdx2];
            Vector3D p2_start(vertices3D[edge2.v1_index].x, vertices3D[edge2.v1_index].y, vertices3D[edge2.v1_index].z);
            Vector3D p2_end(vertices3D[edge2.v2_index].x, vertices3D[edge2.v2_index].y, vertices3D[edge2.v2_index].z);

            Vector3D intersectionPoint;
            if (segmentsIntersect(p1_start, p1_end, p2_start, p2_end, intersectionPoint)) {
                intersectionPoints.push_back(intersectionPoint);
            }
        }
    }

    return !intersectionPoints.empty();
}

void Wireframe::insertCuttingEdgesAndVertices(Plane& plane1, Plane& plane2, const std::vector<Vector3D>& intersectionPoints) {
    // For each intersection point, add a new vertex and update face loops

    for (const Vector3D& point : intersectionPoints) {
        // Check if the vertex already exists
        int vertexIdx = findVertex3DIndex(point.x, point.y, point.z);
        if (vertexIdx == -1) {
            // Add new vertex
            Vertex3D newVertex(point.x, point.y, point.z);
            vertexIdx = vertices3D.size();
            vertices3D.push_back(newVertex);
        }

        // Insert cutting edges into face loops
        // For plane1
        insertVertexIntoFaceLoop(plane1, point, vertexIdx);
        // For plane2
        insertVertexIntoFaceLoop(plane2, point, vertexIdx);
    }
}

void Wireframe::removeFaceLoop(Plane& plane, const std::vector<int>& faceLoop) {
    // Remove the face loop from the plane's faceLoops
    plane.faceLoops.erase(
        std::remove(plane.faceLoops.begin(), plane.faceLoops.end(), faceLoop),
        plane.faceLoops.end()
    );
}

bool Wireframe::segmentsIntersect(const Vector3D& p1_start, const Vector3D& p1_end, const Vector3D& p2_start, const Vector3D& p2_end, Vector3D& intersectionPoint) {
    // Use the 3D line segment intersection algorithm

    // Represent segments as parametric equations
    Vector3D u = p1_end - p1_start;
    Vector3D v = p2_end - p2_start;
    Vector3D w = p1_start - p2_start;

    float a = u.dot(u); // always >= 0
    float b = u.dot(v);
    float c = v.dot(v); // always >= 0
    float d = u.dot(w);
    float e = v.dot(w);

    float D = a * c - b * b; // always >= 0

    float sc, tc;

    // Compute the line parameters of the two closest points
    if (D < 1e-6f) {
        // Lines are almost parallel
        return false; // No intersection
    } else {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }

    // Compute the closest points
    Vector3D closestPoint1 = p1_start + u * sc;
    Vector3D closestPoint2 = p2_start + v * tc;

    // Check if the closest points are the same (within tolerance)
    Vector3D diff = closestPoint1 - closestPoint2;
    if (diff.length() < 1e-6f) {
        // Check if sc and tc are within [0,1]
        if (sc >= 0.0f && sc <= 1.0f && tc >= 0.0f && tc <= 1.0f) {
            intersectionPoint = closestPoint1;
            return true;
        }
    }

    return false; // No intersection
}

void Wireframe::insertVertexIntoFaceLoop(Plane& plane, const Vector3D& point, int vertexIdx) {
    // For each face loop in the plane
    for (auto& faceLoop : plane.faceLoops) {
        // For each edge in the face loop
        for (size_t i = 0; i < faceLoop.size(); ++i) {
            int edgeIdx = faceLoop[i];
            Edge3D& edge = edges3D[edgeIdx];
            Vector3D p1(vertices3D[edge.v1_index].x, vertices3D[edge.v1_index].y, vertices3D[edge.v1_index].z);
            Vector3D p2(vertices3D[edge.v2_index].x, vertices3D[edge.v2_index].y, vertices3D[edge.v2_index].z);

            // Check if the point lies on this edge
            if (pointOnSegment(point, p1, p2)) {
                // Split the edge into two edges at the new vertex
                // Invalidate the old edge
                edge.isValid = false;

                // Create two new edges
                int newEdgeIdx1 = edges3D.size();
                edges3D.emplace_back(edge.v1_index, vertexIdx);
                vertices3D[edge.v1_index].addConnectedEdge(newEdgeIdx1);
                vertices3D[vertexIdx].addConnectedEdge(newEdgeIdx1);

                int newEdgeIdx2 = edges3D.size();
                edges3D.emplace_back(vertexIdx, edge.v2_index);
                vertices3D[vertexIdx].addConnectedEdge(newEdgeIdx2);
                vertices3D[edge.v2_index].addConnectedEdge(newEdgeIdx2);

                // Replace the old edge in the face loop with the two new edges
                faceLoop.erase(faceLoop.begin() + i);
                faceLoop.insert(faceLoop.begin() + i, newEdgeIdx2);
                faceLoop.insert(faceLoop.begin() + i, newEdgeIdx1);

                // Adjust the index to account for the inserted edge
                ++i;

                break; // Move to the next face loop
            }
        }
    }
}

bool Wireframe::pointOnSegment(const Vector3D &point, const Vector3D &segStart, const Vector3D &segEnd) const {
    // Vector from segStart to point
    Vector3D d1 = point - segStart;
    // Vector from segStart to segEnd
    Vector3D d2 = segEnd - segStart;

    // Calculate dot product
    float dotProduct = d1.dot(d2);

    // Check if point lies within the segment bounds
    if (dotProduct < 0.0f || dotProduct > d2.dot(d2)) {
        return false; // Point is outside the segment
    }

    // Check if the point is colinear with the segment
    Vector3D crossProduct = d1.cross(d2);
    return crossProduct.length() < 1e-6f; // Tolerance for floating-point precision
}

// In wireframe.cpp

void Wireframe::generateBodyLoops() {


    std::vector<std::vector<int>> chi; // chi[s][+/-] = 0 or 1 (unused or used)
    std::vector<std::vector<bool>> expanded; // expanded[s][+/-] = true or false

    // Initialize chi and expanded for each face loop side
    // Since each face loop has two sides, we'll represent them with a struct
    std::vector<FaceLoopSide> faceLoopSides;

    for (size_t planeIdx = 0; planeIdx < planes.size(); ++planeIdx) {
        const Plane& plane = planes[planeIdx];
        for (size_t faceLoopIdx = 0; faceLoopIdx < plane.faceLoops.size(); ++faceLoopIdx) {
            // Positive side
            faceLoopSides.push_back({static_cast<int>(planeIdx), static_cast<int>(faceLoopIdx), true});
            // Negative side
            faceLoopSides.push_back({static_cast<int>(planeIdx), static_cast<int>(faceLoopIdx), false});
        }
    }

    std::unordered_map<int, int> chiMap; // Key: faceLoopSideIndex, Value: 0 or 1
    std::unordered_map<int, bool> expandedMap; // Key: faceLoopSideIndex, Value: expanded or not

    for (size_t i = 0; i < faceLoopSides.size(); ++i) {
        chiMap[i] = 0;       // Mark all sides as unused
        expandedMap[i] = false; // Mark all sides as unexpanded
    }

    // B1: Initialize S(S) = empty
    std::vector<int> S; // Set of face loop sides involved in current body loop

    // B6: Repeat until all face loops have been visited on both sides
    while (true) {
        // B2: Select a starting face loop side
        int startingFaceLoopSideIndex = -1;
        for (size_t i = 0; i < faceLoopSides.size(); ++i) {
            if (chiMap[i] == 0) {
                startingFaceLoopSideIndex = static_cast<int>(i);
                break;
            }
        }

        if (startingFaceLoopSideIndex == -1) {
            // All face loops have been visited on both sides
            break;
        }

        // Begin a new body loop
        S.clear();

        // Mark the starting face loop side as used
        chiMap[startingFaceLoopSideIndex] = 1;
        S.push_back(startingFaceLoopSideIndex);
        expandedMap[startingFaceLoopSideIndex] = false; // Mark as unexpanded

        // B4: Repeat until no new face loops to expand
        bool canExpand = true;
        while (canExpand) {
            canExpand = false;
            // B3: Select an unexpanded face loop side from S
            int currentFaceLoopSideIndex = -1;
            for (int idx : S) {
                if (!expandedMap[idx]) {
                    currentFaceLoopSideIndex = idx;
                    break;
                }
            }

            if (currentFaceLoopSideIndex == -1) {
                // No unexpanded face loops in S
                break;
            }

            expandedMap[currentFaceLoopSideIndex] = true;

            // Get the current face loop side
            FaceLoopSide& currentSide = faceLoopSides[currentFaceLoopSideIndex];
            Plane& currentPlane = planes[currentSide.planeIndex];
            std::vector<int>& currentFaceLoop = currentPlane.faceLoops[currentSide.faceLoopIndex];

            // Compute successive face loops for each edge on the boundary
            for (int edgeIdx : currentFaceLoop) {
                Edge3D& edge = edges3D[edgeIdx];

                // Find adjacent face loops at this edge
                std::vector<std::pair<int, int>> adjacentFaceLoops; // Pair of plane index and face loop index

                // Iterate through planes and face loops to find adjacent face loops sharing this edge
                for (size_t planeIdx = 0; planeIdx < planes.size(); ++planeIdx) {
                    Plane& plane = planes[planeIdx];
                    for (size_t faceLoopIdx = 0; faceLoopIdx < plane.faceLoops.size(); ++faceLoopIdx) {
                        if (planeIdx == currentSide.planeIndex && faceLoopIdx == currentSide.faceLoopIndex) {
                            continue; // Skip current face loop
                        }

                        std::vector<int>& faceLoop = plane.faceLoops[faceLoopIdx];
                        if (std::find(faceLoop.begin(), faceLoop.end(), edgeIdx) != faceLoop.end()) {
                            adjacentFaceLoops.push_back({static_cast<int>(planeIdx), static_cast<int>(faceLoopIdx)});
                        }
                    }
                }

                // There should be at least one adjacent face loop
                if (adjacentFaceLoops.empty()) {
                    continue; // No adjacent face loops at this edge
                }

                // Compute the angle and side selection for each adjacent face loop
                double smallestAlpha = 2 * M_PI + 1; // Initialize to a value larger than 2π
                int selectedFaceLoopSideIndex = -1;

                for (const auto& adj : adjacentFaceLoops) {
                    int adjPlaneIdx = adj.first;
                    int adjFaceLoopIdx = adj.second;

                    Plane& adjPlane = planes[adjPlaneIdx];
                    std::vector<int>& adjFaceLoop = adjPlane.faceLoops[adjFaceLoopIdx];

                    // Compute normal vectors
                    Vector3D n_k(currentPlane.a, currentPlane.b, currentPlane.c);
                    Vector3D n_s(adjPlane.a, adjPlane.b, adjPlane.c);

                    // Compute θ
                    double dotProduct = n_k.dot(n_s);
                    double magnitudeProduct = n_k.length() * n_s.length();
                    double cosTheta = dotProduct / magnitudeProduct;
                    // Clamp cosTheta to [-1, 1] to avoid NaNs due to numerical errors
                    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
                    double theta = std::acos(cosTheta);

                    // Compute e (edge direction vector)
                    Vertex3D& v1 = vertices3D[edge.v1_index];
                    Vertex3D& v2 = vertices3D[edge.v2_index];
                    Vector3D e(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
                    e = e.normalize();

                    // Ensure e satisfies the right-hand rule with n_k
                    Vector3D cross_nk_e = n_k.cross(e);
                    if (cross_nk_e.length() < 1e-6) {
                        // e is parallel to n_k, adjust e
                        e = Vector3D(-e.y, e.x, e.z); // Arbitrary orthogonal vector
                    }

                    // Determine the side of the adjacent face loop
                    bool positiveSide = true; // Assume positive side initially

                    // For simplicity, let's assume that the face loop sides are determined by the plane normal
                    // In practice, you might need to implement a function to determine the side based on orientation

                    // Determine α and side selection based on criteria
                    Vector3D cross_ns_nk = n_s.cross(n_k);
                    bool sameDirection = cross_ns_nk.dot(e) > 0;

                    double alpha;
                    bool adjPositiveSide;

                    if (sameDirection && positiveSide) {
                        alpha = theta;
                        adjPositiveSide = true;
                    } else if (!sameDirection && !positiveSide) {
                        alpha = theta;
                        adjPositiveSide = false;
                    } else if (sameDirection && !positiveSide) {
                        alpha = 2 * M_PI - theta;
                        adjPositiveSide = false;
                    } else {
                        alpha = M_PI + theta;
                        adjPositiveSide = true;
                    }

                    // Update if this is the smallest α
                    if (alpha < smallestAlpha) {
                        smallestAlpha = alpha;
                        // Find the index of the adjacent face loop side
                        int adjFaceLoopSideIndex = -1;
                        for (size_t idx = 0; idx < faceLoopSides.size(); ++idx) {
                            FaceLoopSide& side = faceLoopSides[idx];
                            if (side.planeIndex == adjPlaneIdx && side.faceLoopIndex == adjFaceLoopIdx && side.positiveSide == adjPositiveSide) {
                                adjFaceLoopSideIndex = static_cast<int>(idx);
                                break;
                            }
                        }
                        selectedFaceLoopSideIndex = adjFaceLoopSideIndex;
                    }
                }

                if (selectedFaceLoopSideIndex != -1) {
                    // Add the selected face loop side to S if not already in S
                    if (chiMap[selectedFaceLoopSideIndex] == 0) {
                        chiMap[selectedFaceLoopSideIndex] = 1;
                        S.push_back(selectedFaceLoopSideIndex);
                        expandedMap[selectedFaceLoopSideIndex] = false;
                        canExpand = true;
                    }
                }
            }
        }

        // B5: Check body-loop legality
        if (isBodyLoopLegal(S, faceLoopSides)) {
            // Store the closed body loop
            std::vector<int> bodyLoop;
            for (int idx : S) {
                FaceLoopSide& side = faceLoopSides[idx];
                // We can represent the face loop side uniquely with its index
                bodyLoop.push_back(idx);
            }
            bodyLoops.push_back(bodyLoop);
        }

        // B6: Prepare for the next body loop search
        // The loop will continue automatically as we check for unvisited face loop sides
    }
}

bool Wireframe::isBodyLoopLegal(const std::vector<int>& S, const std::vector<FaceLoopSide>& faceLoopSides)const {
    // For every edge involved in this body loop, check if it is shared by exactly two face loops

    // Map from edge index to count of face loops sharing it
    std::unordered_map<int, int> edgeFaceLoopCount;

    for (int idx : S) {
        const FaceLoopSide& side = faceLoopSides[idx];
        const Plane& plane = planes[side.planeIndex];
        const std::vector<int>& faceLoop = plane.faceLoops[side.faceLoopIndex];

        for (int edgeIdx : faceLoop) {
            edgeFaceLoopCount[edgeIdx]++;
        }
    }

    // Check that each edge is shared by exactly two face loops
    for (const auto& entry : edgeFaceLoopCount) {
        if (entry.second != 2) {
            return false; // Not a legal body loop
        }
    }

    return true; // Legal body loop
}

Vector3D Wireframe::getEdgeDirection(const Edge3D& edge, const Vector3D& normal) {
    Vertex3D& v1 = vertices3D[edge.v1_index];
    Vertex3D& v2 = vertices3D[edge.v2_index];
    Vector3D e(v2.x - v1.x, v2.y - v1.y, v2.z - v1.z);
    e = e.normalize();

    // Use the right-hand rule to determine if we need to reverse the edge direction
    Vector3D crossProduct = normal.cross(e);
    if (crossProduct.length() < 1e-6) {
        // Normal and edge are parallel
        return e;
    }

    // If the cross product points in the negative z-direction, reverse the edge direction
    if (crossProduct.z < 0) {
        e = Vector3D(-e.x, -e.y, -e.z);
    }

    return e;
}

bool Wireframe::determineFaceLoopSide(const Plane& currentPlane, const Plane& adjPlane, const Edge3D& edge, bool currentPositiveSide, bool& adjPositiveSide) {
    // Compute normals
    Vector3D n_k(currentPlane.a, currentPlane.b, currentPlane.c);
    Vector3D n_s(adjPlane.a, adjPlane.b, adjPlane.c);

    // Compute θ
    double dotProduct = n_k.dot(n_s);
    double magnitudeProduct = n_k.length() * n_s.length();
    double cosTheta = dotProduct / magnitudeProduct;
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    double theta = std::acos(cosTheta);

    // Compute e (edge direction vector)
    Vector3D e = getEdgeDirection(edge, n_k);

    // Compute cross products
    Vector3D cross_ns_nk = n_s.cross(n_k);
    bool sameDirection = cross_ns_nk.dot(e) > 0;

    // Determine α and side selection based on criteria
    double alpha;
    if (sameDirection && currentPositiveSide) {
        alpha = theta;
        adjPositiveSide = true;
    } else if (!sameDirection && !currentPositiveSide) {
        alpha = theta;
        adjPositiveSide = false;
    } else if (sameDirection && !currentPositiveSide) {
        alpha = 2 * M_PI - theta;
        adjPositiveSide = false;
    } else {
        alpha = M_PI + theta;
        adjPositiveSide = true;
    }

    // Return whether adjPositiveSide is determined
    return true;
}





//generate candidate 


void Wireframe::generateCandidateObjects() {
    size_t numBodyLoops = bodyLoops.size();
    size_t numCandidates = (1 << numBodyLoops) - 1; // 2^N - 1

    for (size_t i = 1; i <= numCandidates; ++i) {
        CandidateObject candidate;
        for (size_t j = 0; j < numBodyLoops; ++j) {
            if (i & (1 << j)) {
                candidate.bodyLoops.push_back(bodyLoops[j]);
            }
        }
        candidates.push_back(candidate);
    }
}

void CandidateObject::eliminateRedundantFaceLoops() {
    // For each face loop, check if both sides are selected in the same solid
    // If so, remove it from the face loops of the candidate
    std::vector<FaceLoop> uniqueFaceLoops;
    std::unordered_map<FaceLoop, int> faceLoopCounts;

    for (const auto& bodyLoop : bodyLoops) {
        for (const auto& faceLoop : bodyLoop.faceLoops) {
            faceLoopCounts[faceLoop] += faceLoop.sideSelection;
        }
    }

    for (const auto& [faceLoop, count] : faceLoopCounts) {
        if (abs(count) == 1) {
            uniqueFaceLoops.push_back(faceLoop);
        }
        // Else, faceLoop is redundant and not added
    }

    faceLoops = uniqueFaceLoops;
}

void CandidateObject::eliminateRedundantEdges() {
    // Identify edges shared by two and only two coplanar face loops
    // Remove such edges and merge the face loops

    // Implementation depends on your data structures
}
bool CandidateObject::isLegal() {
    // Check for shared vertices without shared edges
    if (hasIllegalVertexSharing()) {
        return false;
    }

    // Check for edges with more than two adjacent face loops
    if (hasEdgeWithTooManyFaces()) {
        return false;
    }

    return true;
}

bool CandidateObject::hasIllegalVertexSharing() {
    // Implement logic to detect face loops that share vertices but not edges
}

bool CandidateObject::hasEdgeWithTooManyFaces() {
    // Implement logic to count the number of face loops adjacent to each edge
}
void CandidateObject::projectOntoPlanes(Projection2D& frontView, Projection2D& topView, Projection2D& sideView) {
    // For each edge in the candidate object, project it onto the three planes
    for (const auto& edge : edges) {
        Vertex3D v1 = vertices[edge.v1_index];
        Vertex3D v2 = vertices[edge.v2_index];

        // Project onto front view (xz-plane)
        frontView.addEdge(v1.getX(), v1.getZ(), v2.getX(), v2.getZ());

        // Project onto top view (xy-plane)
        topView.addEdge(v1.getX(), v1.getY(), v2.getX(), v2.getY());

        // Project onto side view (yz-plane)
        sideView.addEdge(v1.getY(), v1.getZ(), v2.getY(), v2.getZ());
    }
}

bool CandidateObject::isConsistentWithViews(const Projection2D& inputFrontView, const Projection2D& inputTopView, const Projection2D& inputSideView) {
    Projection2D candidateFrontView, candidateTopView, candidateSideView;
    projectOntoPlanes(candidateFrontView, candidateTopView, candidateSideView);

    // Compare candidate projections with input views
    return candidateFrontView.matches(inputFrontView) &&
           candidateTopView.matches(inputTopView) &&
           candidateSideView.matches(inputSideView);
}
void Wireframe::processCandidates(const Projection2D& inputFrontView, const Projection2D& inputTopView, const Projection2D& inputSideView) {
    for (auto& candidate : candidates) {
        candidate.eliminateRedundantFaceLoops();
        candidate.eliminateRedundantEdges();

        if (!candidate.isLegal()) {
            continue; // Skip illegal candidates
        }

        if (candidate.isConsistentWithViews(inputFrontView, inputTopView, inputSideView)) {
            validCandidates.push_back(candidate);
        }
    }

    // Now validCandidates contains all the correct solutions
}
