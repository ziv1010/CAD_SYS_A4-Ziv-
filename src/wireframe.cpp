#include "wireframe.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

// Helper function to compare floating-point numbers
bool WireframeConstructor::isClose(float a, float b, float tol) {
    return fabs(a - b) < tol;
}

// Helper function to find the index of a vertex in a list based on coordinates
int WireframeConstructor::findVertexIndex(const std::vector<Vertex2D>& v_list, float x, float y) {
    for (size_t i = 0; i < v_list.size(); ++i) {
        if (isClose(v_list[i].x, x) && isClose(v_list[i].y, y)) {
            return static_cast<int>(i);
        }
    }
    return -1; // Not found
}

// Read input data from file
void WireframeConstructor::readInput(const std::string& filename) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open input file " << filename << std::endl;
        return;
    }

    int N;
    infile >> N;

    for (int i = 0; i < N; ++i) {
        float tx, ty, fx, fy, sx, sy;
        infile >> tx >> ty >> fx >> fy >> sx >> sy;

        v_list_t.emplace_back(tx, ty);
        v_list_f.emplace_back(fx, fy);
        v_list_s.emplace_back(sx, sy);
    }

    int Te;
    infile >> Te;

    for (int i = 0; i < Te; ++i) {
        float x0, y0, x1, y1;
        infile >> x0 >> y0 >> x1 >> y1;

        int idx0 = findVertexIndex(v_list_t, x0, y0);
        int idx1 = findVertexIndex(v_list_t, x1, y1);

        if (idx0 == -1 || idx1 == -1) {
            std::cerr << "Error: Edge in top view refers to unknown vertices." << std::endl;
        } else {
            e_list_t.emplace_back(idx0, idx1);
        }
    }

    int Fe;
    infile >> Fe;

    for (int i = 0; i < Fe; ++i) {
        float x0, y0, x1, y1;
        infile >> x0 >> y0 >> x1 >> y1;

        int idx0 = findVertexIndex(v_list_f, x0, y0);
        int idx1 = findVertexIndex(v_list_f, x1, y1);

        if (idx0 == -1 || idx1 == -1) {
            std::cerr << "Error: Edge in front view refers to unknown vertices." << std::endl;
        } else {
            e_list_f.emplace_back(idx0, idx1);
        }
    }

    int Se;
    infile >> Se;

    for (int i = 0; i < Se; ++i) {
        float x0, y0, x1, y1;
        infile >> x0 >> y0 >> x1 >> y1;

        int idx0 = findVertexIndex(v_list_s, x0, y0);
        int idx1 = findVertexIndex(v_list_s, x1, y1);

        if (idx0 == -1 || idx1 == -1) {
            std::cerr << "Error: Edge in side view refers to unknown vertices." << std::endl;
        } else {
            e_list_s.emplace_back(idx0, idx1);
        }
    }

    infile.close();
}

// Generate potential 3D edges and vertices
void WireframeConstructor::generatePotentialEdgesAndVertices() {
    int N = v_list_f.size();
    for (int i = 0; i < N; ++i) {
        float X_f = v_list_f[i].x;
        float Y_f = v_list_f[i].y;

        float X_t = v_list_t[i].x;
        float Z_t = v_list_t[i].y;

        float Z_s = v_list_s[i].x;
        float Y_s = v_list_s[i].y;

        // Check consistency
        if (!isClose(X_f, X_t) || !isClose(Y_f, Y_s) || !isClose(Z_t, Z_s)) {
            std::cerr << "Inconsistent projections for vertex " << i << std::endl;
        }

        // Reconstruct 3D vertex
        float X = X_f;
        float Y = Y_f;
        float Z = Z_t;

        p_vertices.emplace_back(X, Y, Z);
    }

    // Generate potential edges from the 2D edges
    // For edges in front view
    for (const auto& edge_f : e_list_f) {
        int idx0 = edge_f.start_vertex_index;
        int idx1 = edge_f.end_vertex_index;
        p_edges.emplace_back(idx0, idx1);
    }

    // For edges in top view
    for (const auto& edge_t : e_list_t) {
        int idx0 = edge_t.start_vertex_index;
        int idx1 = edge_t.end_vertex_index;

        // Check if this edge already exists
        bool exists = false;
        for (const auto& edge : p_edges) {
            if ((edge.start_vertex_index == idx0 && edge.end_vertex_index == idx1) ||
                (edge.start_vertex_index == idx1 && edge.end_vertex_index == idx0)) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            p_edges.emplace_back(idx0, idx1);
        }
    }

    // For edges in side view
    for (const auto& edge_s : e_list_s) {
        int idx0 = edge_s.start_vertex_index;
        int idx1 = edge_s.end_vertex_index;

        // Check if this edge already exists
        bool exists = false;
        for (const auto& edge : p_edges) {
            if ((edge.start_vertex_index == idx0 && edge.end_vertex_index == idx1) ||
                (edge.start_vertex_index == idx1 && edge.end_vertex_index == idx0)) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            p_edges.emplace_back(idx0, idx1);
        }
    }
}

// Check if two 3D edges are colinear and overlapping
bool WireframeConstructor::areEdgesColinearAndOverlapping(const Edge3D& e1, const Edge3D& e2) {
    const Vertex3D& p1 = p_vertices[e1.start_vertex_index];
    const Vertex3D& p2 = p_vertices[e1.end_vertex_index];
    const Vertex3D& q1 = p_vertices[e2.start_vertex_index];
    const Vertex3D& q2 = p_vertices[e2.end_vertex_index];

    // Compute direction vectors
    float dx1 = p2.x - p1.x;
    float dy1 = p2.y - p1.y;
    float dz1 = p2.z - p1.z;

    float dx2 = q2.x - q1.x;
    float dy2 = q2.y - q1.y;
    float dz2 = q2.z - q1.z;

    // Compute cross product
    float cx = dy1 * dz2 - dz1 * dy2;
    float cy = dz1 * dx2 - dx1 * dz2;
    float cz = dx1 * dy2 - dy1 * dx2;

    if (!isClose(cx, 0.0f) || !isClose(cy, 0.0f) || !isClose(cz, 0.0f)) {
        return false; // Not colinear
    }

    // Check if they lie on the same line
    float dx_p1q1 = q1.x - p1.x;
    float dy_p1q1 = q1.y - p1.y;
    float dz_p1q1 = q1.z - p1.z;

    float cpx = dy1 * dz_p1q1 - dz1 * dy_p1q1;
    float cpy = dz1 * dx_p1q1 - dx1 * dz_p1q1;
    float cpz = dx1 * dy_p1q1 - dy1 * dx_p1q1;

    if (!isClose(cpx, 0.0f) || !isClose(cpy, 0.0f) || !isClose(cpz, 0.0f)) {
        return false; // Not colinear
    }

    return true; // Colinear
}

// Check if two colinear edges overlap
bool WireframeConstructor::edgesOverlap(const Edge3D& e1, const Edge3D& e2) {
    const Vertex3D& p1 = p_vertices[e1.start_vertex_index];
    const Vertex3D& p2 = p_vertices[e1.end_vertex_index];
    const Vertex3D& q1 = p_vertices[e2.start_vertex_index];
    const Vertex3D& q2 = p_vertices[e2.end_vertex_index];

    // Decide which coordinate to use
    float dx = fabs(p2.x - p1.x);
    float dy = fabs(p2.y - p1.y);
    float dz = fabs(p2.z - p1.z);
    char coord = (dx >= dy && dx >= dz) ? 'x' : (dy >= dx && dy >= dz) ? 'y' : 'z';

    float p_min, p_max, q_min, q_max;
    if (coord == 'x') {
        p_min = std::min(p1.x, p2.x);
        p_max = std::max(p1.x, p2.x);
        q_min = std::min(q1.x, q2.x);
        q_max = std::max(q1.x, q2.x);
    } else if (coord == 'y') {
        p_min = std::min(p1.y, p2.y);
        p_max = std::max(p1.y, p2.y);
        q_min = std::min(q1.y, q2.y);
        q_max = std::max(q1.y, q2.y);
    } else {
        p_min = std::min(p1.z, p2.z);
        p_max = std::max(p1.z, p2.z);
        q_min = std::min(q1.z, q2.z);
        q_max = std::max(q1.z, q2.z);
    }

    return !(p_max < q_min || q_max < p_min);
}

// Remove overlapping edges (RER procedure)
void WireframeConstructor::removeOverlappingEdges() {
    // Initially, mark all edges as unexamined
    for (auto& edge : p_edges) {
        edge.examined = false;
    }

    size_t num_edges = p_edges.size();

    for (size_t i = 0; i < num_edges; ++i) {
        if (p_edges[i].examined) continue;

        Edge3D& e_i = p_edges[i];
        std::vector<size_t> E_indices; // Indices of edges in E
        E_indices.push_back(i);

        for (size_t j = 0; j < num_edges; ++j) {
            if (j == i) continue;
            Edge3D& e_j = p_edges[j];

            if (areEdgesColinearAndOverlapping(e_i, e_j)) {
                bool overlaps_with_E = false;
                for (size_t k : E_indices) {
                    if (edgesOverlap(p_edges[k], e_j)) {
                        overlaps_with_E = true;
                        break;
                    }
                }
                if (overlaps_with_E) {
                    E_indices.push_back(j);
                }
            }
        }

        if (E_indices.size() == 1) {
            e_i.examined = true;
        } else {
            // Remove overlapping edges
            std::vector<int> vertex_indices;

            for (size_t idx : E_indices) {
                Edge3D& e = p_edges[idx];
                vertex_indices.push_back(e.start_vertex_index);
                vertex_indices.push_back(e.end_vertex_index);
                e.examined = true;
            }

            // Remove edges in E from p_edges
            for (size_t idx : E_indices) {
                p_edges[idx].examined = true;
            }

            // Sort vertices along the dominant axis
            float dx = fabs(p_vertices[e_i.end_vertex_index].x - p_vertices[e_i.start_vertex_index].x);
            float dy = fabs(p_vertices[e_i.end_vertex_index].y - p_vertices[e_i.start_vertex_index].y);
            float dz = fabs(p_vertices[e_i.end_vertex_index].z - p_vertices[e_i.start_vertex_index].z);
            char coord = (dx >= dy && dx >= dz) ? 'x' : (dy >= dx && dy >= dz) ? 'y' : 'z';

            // Remove duplicates
            std::sort(vertex_indices.begin(), vertex_indices.end());
            vertex_indices.erase(std::unique(vertex_indices.begin(), vertex_indices.end()), vertex_indices.end());

            // Sort vertices
            std::sort(vertex_indices.begin(), vertex_indices.end(), [&](int a, int b) {
                const Vertex3D& va = p_vertices[a];
                const Vertex3D& vb = p_vertices[b];
                return (coord == 'x') ? va.x < vb.x : (coord == 'y') ? va.y < vb.y : va.z < vb.z;
            });

            // Add new edges
            for (size_t k = 0; k + 1 < vertex_indices.size(); ++k) {
                p_edges.emplace_back(vertex_indices[k], vertex_indices[k + 1]);
                p_edges.back().examined = true;
            }
        }
    }

    // Remove examined edges
    p_edges.erase(std::remove_if(p_edges.begin(), p_edges.end(), [](const Edge3D& e) {
        return e.examined;
    }), p_edges.end());
}

// Check if two edges are colinear (used in PEVR)
bool WireframeConstructor::areEdgesColinear(const Edge3D& e1, const Edge3D& e2, int common_vertex_index) {
    int v1 = (e1.start_vertex_index == common_vertex_index) ? e1.end_vertex_index : e1.start_vertex_index;
    int v2 = (e2.start_vertex_index == common_vertex_index) ? e2.end_vertex_index : e2.start_vertex_index;

    const Vertex3D& p_common = p_vertices[common_vertex_index];
    const Vertex3D& p1 = p_vertices[v1];
    const Vertex3D& p2 = p_vertices[v2];

    // Vectors
    float dx1 = p1.x - p_common.x;
    float dy1 = p1.y - p_common.y;
    float dz1 = p1.z - p_common.z;

    float dx2 = p2.x - p_common.x;
    float dy2 = p2.y - p_common.y;
    float dz2 = p2.z - p_common.z;

    // Cross product
    float cx = dy1 * dz2 - dz1 * dy2;
    float cy = dz1 * dx2 - dx1 * dz2;
    float cz = dx1 * dy2 - dy1 * dx2;

    return isClose(cx, 0.0f) && isClose(cy, 0.0f) && isClose(cz, 0.0f);
}

// Remove pathological edges and vertices (PEVR procedure)
void WireframeConstructor::removePathologicalVerticesAndEdges() {
    // Compute edge count for each vertex
    std::vector<int> edge_count(p_vertices.size(), 0);
    for (const auto& edge : p_edges) {
        edge_count[edge.start_vertex_index]++;
        edge_count[edge.end_vertex_index]++;
    }

    // Keep track of vertices and edges to remove
    std::vector<bool> vertex_to_remove(p_vertices.size(), false);
    std::vector<bool> edge_to_remove(p_edges.size(), false);

    // Process vertices
    for (size_t i = 0; i < p_vertices.size(); ++i) {
        int count = edge_count[i];

        if (count == 0) {
            // Isolated vertex
            vertex_to_remove[i] = true;
        } else if (count == 1) {
            // Dangling edge
            vertex_to_remove[i] = true;
            for (size_t j = 0; j < p_edges.size(); ++j) {
                const Edge3D& edge = p_edges[j];
                if (edge.start_vertex_index == i || edge.end_vertex_index == i) {
                    edge_to_remove[j] = true;
                    break;
                }
            }
        } else if (count == 2) {
            // Check colinearity
            std::vector<size_t> adjacent_edges;
            for (size_t j = 0; j < p_edges.size(); ++j) {
                const Edge3D& edge = p_edges[j];
                if ((edge.start_vertex_index == i || edge.end_vertex_index == i) && !edge_to_remove[j]) {
                    adjacent_edges.push_back(j);
                }
            }

            if (adjacent_edges.size() != 2) continue;

            const Edge3D& e1 = p_edges[adjacent_edges[0]];
            const Edge3D& e2 = p_edges[adjacent_edges[1]];

            if (areEdgesColinear(e1, e2, i)) {
                int v1 = (e1.start_vertex_index == i) ? e1.end_vertex_index : e1.start_vertex_index;
                int v2 = (e2.start_vertex_index == i) ? e2.end_vertex_index : e2.start_vertex_index;

                p_edges.emplace_back(v1, v2);
                edge_to_remove[adjacent_edges[0]] = true;
                edge_to_remove[adjacent_edges[1]] = true;
                vertex_to_remove[i] = true;
            } else {
                edge_to_remove[adjacent_edges[0]] = true;
                edge_to_remove[adjacent_edges[1]] = true;
                vertex_to_remove[i] = true;
            }
        }
        // Additional cases for count == 3 and count == 4 can be implemented similarly
    }

    // Remove edges
    std::vector<Edge3D> new_edges;
    for (size_t i = 0; i < p_edges.size(); ++i) {
        if (!edge_to_remove[i]) {
            new_edges.push_back(p_edges[i]);
        }
    }
    p_edges = new_edges;

    // Remove vertices and update indices
    std::vector<Vertex3D> new_vertices;
    std::vector<int> index_mapping(p_vertices.size(), -1);
    int new_index = 0;
    for (size_t i = 0; i < p_vertices.size(); ++i) {
        if (!vertex_to_remove[i]) {
            new_vertices.push_back(p_vertices[i]);
            index_mapping[i] = new_index++;
        }
    }

    // Update edges with new indices
    for (auto& edge : p_edges) {
        edge.start_vertex_index = index_mapping[edge.start_vertex_index];
        edge.end_vertex_index = index_mapping[edge.end_vertex_index];
    }

    p_vertices = new_vertices;
}

// Construct wireframe by executing all steps
void WireframeConstructor::constructWireframe() {
    generatePotentialEdgesAndVertices();
    removeOverlappingEdges();
    removePathologicalVerticesAndEdges();
}

// Write output to file
void WireframeConstructor::writeOutput(const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Cannot open output file " << filename << std::endl;
        return;
    }

    // Write vertices
    for (size_t i = 0; i < p_vertices.size(); ++i) {
        const Vertex3D& v = p_vertices[i];
        outfile << "v " << v.x << " " << v.y << " " << v.z << std::endl;
    }

    // Write edges
    for (const auto& edge : p_edges) {
        outfile << "e " << edge.start_vertex_index + 1 << " " << edge.end_vertex_index + 1 << std::endl;
    }

    outfile.close();
}