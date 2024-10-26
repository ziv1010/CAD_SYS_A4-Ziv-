#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <vector>
#include "vertex.h"
#include "edge.h"
#include "face.h"
#include "polygon2d.h"

class Object3D {
public:
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;

    // Add a vertex
    void addVertex(const Vertex& v) {
        vertices.push_back(v);
    }

    // Add an edge
    void addEdge(const Edge& e) {
        edges.push_back(e);
    }

    // Add a face
    void addFace(const Face& f) {
        faces.push_back(f);
    }

    // Process edge intersections to handle hidden lines
    void processEdgeIntersections();

    float computeSurfaceArea() const;
    float computeVolume() const;


private:
    // Helper functions
    std::pair<Vertex, Vertex> projectEdgeToPlane(const Edge& edge, const std::string& plane) const;
    bool edgesIntersect2D(const Vertex& p1, const Vertex& p2,
                          const Vertex& q1, const Vertex& q2,
                          Vertex& intersection) const;
    Vertex compute3DIntersectionPoint(const Edge& edge, const Vertex& intersection2D, const std::string& plane) const;
};

#endif // OBJECT3D_H