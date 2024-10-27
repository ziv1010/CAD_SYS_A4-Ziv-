#ifndef WIREFRAME_MODEL_H
#define WIREFRAME_MODEL_H

#include "vertex3d.h"
#include "edge3d.h"
#include "face3d.h"
#include "graph2d.h"
#include <vector>
#include <string>

class WireframeModel {
public:
    std::vector<Vertex3D> probableVertices;
    std::vector<Edge3D> probableEdges;
    std::vector<Face3D> probableFaces;

    // Public methods
    void generateProbableVertices(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView);
    void generateProbableEdges(const Graph2D& topView, const Graph2D& frontView, const Graph2D& sideView);
    void validateVerticesAndEdges();
    void generateProbableFaces();
    void removePseudoElements();
    void writeToFile(const std::string& filename) const;
    void print() const;

private:
    // Helper functions
    bool isPointOnLineSegment(const Vertex3D& p, const Vertex3D& a, const Vertex3D& b) const;
    bool isEdgeContained(const Edge3D& e1, const Edge3D& e2) const;
    std::vector<int> getAdjacentEdges(int vertexIdx) const;
    float computeInternalAngle(const Vertex3D& v, const Vertex3D& v1, const Vertex3D& v2) const;
    Vertex3D edgeVector(int startIdx, int endIdx) const;
    Vertex3D crossProduct(const Vertex3D& a, const Vertex3D& b) const;
    bool areFacesCoplanar(const Face3D& face1, const Face3D& face2);
    void mergeFaces(Face3D& face1, Face3D& face2);
    bool applyDecisionRules();
    bool decisionChaining();
    bool verifyMoebiusRule();

    // Rule functions (declare them as member functions)
    bool applyRule1(Edge3D& edge, std::vector<Face3D>& faces);
    bool applyRule4(Edge3D& edge, std::vector<Face3D>& faces);
    bool applyRule5(Edge3D& edge, std::vector<Face3D>& faces);
    bool applyRule6(Edge3D& edge, std::vector<Face3D>& faces);
};

#endif // WIREFRAME_MODEL_H
