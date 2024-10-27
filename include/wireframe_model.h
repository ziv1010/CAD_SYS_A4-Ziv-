#ifndef WIREFRAME_MODEL_H
#define WIREFRAME_MODEL_H

#include "vertex3d.h"
#include "graph2d.h"
#include <vector>

class WireframeModel {
public:
    std::vector<Vertex3D> probableVertices;

    // Method to generate probable 3D vertices
    void generateProbableVertices(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView);

    // Method to write probable vertices to output file
    void writeVerticesToFile(const std::string& filename) const;

    // Debug print
    void print() const;
};

#endif // WIREFRAME_MODEL_H