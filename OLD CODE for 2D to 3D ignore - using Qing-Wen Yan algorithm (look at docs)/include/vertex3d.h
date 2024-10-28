#ifndef VERTEX3D_H
#define VERTEX3D_H

#include <vector>
#include <algorithm>

class Vertex3D {
public:
    float x, y, z;
    std::vector<int> connectedEdges; // Indices of connected edges in the Wireframe's edges3D list

    // Constructor
    Vertex3D(float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f)
        : x(x_), y(y_), z(z_) {}

    // Accessors
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    // Mutators
    void setX(float x_) { x = x_; }
    void setY(float y_) { y = y_; }
    void setZ(float z_) { z = z_; }

    // Add a connected edge
    void addConnectedEdge(int edgeIndex) {
        connectedEdges.push_back(edgeIndex);
    }

    // Remove a connected edge
    void removeConnectedEdge(int edgeIndex) {
        connectedEdges.erase(
            std::remove(connectedEdges.begin(), connectedEdges.end(), edgeIndex),
            connectedEdges.end()
        );
    }

    // Get degree (number of connected edges)
    int degree() const { return connectedEdges.size(); }
};

#endif // VERTEX3D_H
