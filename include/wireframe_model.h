#ifndef WIREFRAME_MODEL_H
#define WIREFRAME_MODEL_H

#include "vertex3d.h"
#include "edge3d.h"
#include "graph2d.h"
#include <vector>
#include <string>

class WireframeModel {
public:
    std::vector<Vertex3D> probableVertices;
    std::vector<Edge3D> probableEdges;

    // Method to generate probable 3D vertices
    void generateProbableVertices(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView);

    // Method to generate probable 3D edges
    void generateProbableEdges(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView);

    // Method to validate p-vertices and p-edges
    void validateVerticesAndEdges();

    // Method to write probable vertices and edges to output file
    void writeToFile(const std::string& filename) const;

    // Debug print
    void print() const;

private:
    // Helper functions
    bool isPointOnLineSegment(const Vertex3D& p, const Vertex3D& a, const Vertex3D& b) const;
    bool isEdgeContained(const Edge3D& e1, const Edge3D& e2) const;

};

#endif // WIREFRAME_MODEL_H