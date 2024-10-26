// face.h

#ifndef FACE_H
#define FACE_H

#include <vector>
#include "vertex.h"
#include "polygon2d.h"  // Make sure to include this

class Face {
public:
    std::vector<int> vertexIndices; // Indices of vertices forming the face, in order

    // Constructor
    Face(const std::vector<int>& indices) : vertexIndices(indices) {}

    // Method to get vertex indices
    std::vector<int> getVertexIndices() const {
        return vertexIndices;
    }

    // **Add this declaration**
    // Project the face onto a specified plane and return as Polygon2D
    Polygon2D projectFaceToPlane(const std::string& plane, const std::vector<Vertex>& vertices) const;
};

#endif // FACE_H