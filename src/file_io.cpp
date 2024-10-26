#include "file_io.h"
#include "object3d.h"  // Make sure to include the definition of Object3D
#include <fstream>
#include <sstream>
#include <iostream>

void read3DObjectFromFile(const std::string& filename, Object3D& object) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open input file " << filename << std::endl;
        return;
    }

    int numVertices;
    infile >> numVertices;

    object.vertices.clear();
    for (int i = 0; i < numVertices; ++i) {
        float x, y, z;
        infile >> x >> y >> z;
        object.addVertex(Vertex(x, y, z));
    }

    int numEdges;
    infile >> numEdges;

    object.edges.clear();
    for (int i = 0; i < numEdges; ++i) {
        int v1, v2;
        infile >> v1 >> v2;
        object.addEdge(Edge(v1, v2));
    }

    int numFaces;
    infile >> numFaces;

    object.faces.clear();
    for (int i = 0; i < numFaces; ++i) {
        int numFaceVertices;
        infile >> numFaceVertices;
        std::vector<int> faceVertices;
        for (int j = 0; j < numFaceVertices; ++j) {
            int v;
            infile >> v;
            faceVertices.push_back(v);
        }

        // Create Face with vertex indices
        object.addFace(Face(faceVertices));
    }

    infile.close();
}