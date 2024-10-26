#ifndef WIREFRAME_H
#define WIREFRAME_H

#include <vector>
#include <string>
#include "vertex2d.h"
#include "edge2d.h"
#include "vertex3d.h"
#include "edge3d.h"

class WireframeConstructor {
public:
    // Input 2D vertices and edges for each projection
    std::vector<Vertex2D> v_list_f; // Front view vertices
    std::vector<Vertex2D> v_list_t; // Top view vertices
    std::vector<Vertex2D> v_list_s; // Side view vertices

    std::vector<Edge2D> e_list_f;   // Front view edges
    std::vector<Edge2D> e_list_t;   // Top view edges
    std::vector<Edge2D> e_list_s;   // Side view edges

    // Potential 3D vertices and edges
    std::vector<Vertex3D> p_vertices;
    std::vector<Edge3D> p_edges;

    // Methods
    void readInput(const std::string& filename);
    void constructWireframe();
    void removeOverlappingEdges();
    void removePathologicalVerticesAndEdges();
    void writeOutput(const std::string& filename);

private:
    // Helper methods
    void generatePotentialEdgesAndVertices();
    bool isClose(float a, float b, float tol = 1e-6);
    int findVertexIndex(const std::vector<Vertex2D>& v_list, float x, float y);
    bool areEdgesColinearAndOverlapping(const Edge3D& e1, const Edge3D& e2);
    bool edgesOverlap(const Edge3D& e1, const Edge3D& e2);
    bool areEdgesColinear(const Edge3D& e1, const Edge3D& e2, int common_vertex_index);
};

#endif // WIREFRAME_H