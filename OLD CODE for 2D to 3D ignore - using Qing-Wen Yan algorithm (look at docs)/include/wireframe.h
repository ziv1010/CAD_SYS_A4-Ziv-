#ifndef WIREFRAME_H
#define WIREFRAME_H

#include <vector>
#include "vertex3d.h"    // 3D vertex
#include "edge3d.h"      // 3D edge
#include "vertex2d.h"    // 2D vertex
#include "edge2d.h"      // 2D edge
#include "projection2d.h"
#include "plane.h"       // New: Plane class
#include "vector3d.h"    // New: Vector3D class
#include <unordered_map>
#include "vector2d.h" 
#include "FaceLoopSide.h"
#include "face_loop.h"      // FaceLoop class

class CandidateObject;

struct BodyLoop {
    std::vector<FaceLoop> faceLoops; // Collection of face loops in the body loop

    // Additional members and methods as needed

    // Equality operator for comparison
    bool operator==(const BodyLoop& other) const {
        return faceLoops == other.faceLoops;
    }
};

struct CandidateObject {
    std::vector<BodyLoop> bodyLoops;            // Selected body loops forming the candidate
    std::vector<FaceLoop> uniqueFaceLoops;      // Unique face loops after eliminating redundancies
    std::vector<Edge3D> uniqueEdges;            // Unique edges after eliminating redundancies

    // Constructor
    CandidateObject() = default;

    // Methods to eliminate redundancies and check legality
    void eliminateRedundantFaceLoops();
    void eliminateRedundantEdges();
    bool isLegal() const;
    bool isConsistentWithViews(const Projection2D& inputFrontView,
                               const Projection2D& inputTopView,
                               const Projection2D& inputSideView) const;

private:
    // Helper methods
    bool hasIllegalVertexSharing() const;
    bool hasEdgeWithTooManyFaces() const;
    void projectOntoPlanes(Projection2D& frontView,
                           Projection2D& topView,
                           Projection2D& sideView) const;
};

// Forward declarations if necessary
class Wireframe {
public:
    std::vector<Vertex3D> vertices3D;
    std::vector<Edge3D> edges3D;
    std::vector<Plane> planes; // To store generated planes

    // Constructor
    Wireframe(const Projection2D& front, const Projection2D& top, const Projection2D& side);

    // Generate 3D vertices
    void generate3DVertices();

    // Generate 3D edges
    void generate3DEdges();

    // Remove redundant edges
    void removeRedundantEdges();

    // Remove pathological edges and vertices using PEVR
    void removePathologicalElements();

    // Generate planar graphs
    void generatePlanarGraphs(); // New method

    // Save wireframe to file
    void saveToFile(const std::string& filename) const;

    void generateFaceLoops();

    void handleCuttingEdgesAndVertices();

    void generateBodyLoops();

    void generateCandidateObjects();
    void processCandidates(const Projection2D& inputFrontView,
                           const Projection2D& inputTopView,
                           const Projection2D& inputSideView);
    // In wireframe.h



private:
    const Projection2D& frontView;
    const Projection2D& topView;
    const Projection2D& sideView;

    // Helper functions
    int findVertex3DIndex(float x, float y, float z) const;
    bool areCollinear(int edge1, int edge2) const;
    bool areCoplanar(const std::vector<int>& edgeIndices) const;
    void mergeEdges(int edge1, int edge2);

    // New Helper Functions for PGG
    bool isPlanarGraphClosed(const Plane& plane) const;
    void removePathologicalElementsAtVertex(int vertexIndex);
    void removeDanglingEdgesFromPlane(Plane& plane);
    void adjustIndicesAfterVertexRemoval(int removedVertexIndex);
    void adjustEdgeIndicesAfterEdgeRemoval(int removedEdgeIndex);

   // Helper functions for FLG
    std::vector<int> orderEdgesAtVertex(int vertexIdx, const std::vector<int>& connectedEdges, const Plane& plane);
    bool findBasicLoop(int startEdgeIdx, const Plane& plane,
                    const std::unordered_map<int, std::vector<int>>& orderedEdgesAtVertex,
                    std::vector<bool>& edgeVisited, std::vector<int>& basicLoop);
    std::vector<std::vector<bool>> computeInclusionMatrix(const Plane& plane);
    std::vector<std::vector<int>> formFaceLoops(const std::vector<std::vector<int>>& basicLoops,
                                                const std::vector<std::vector<bool>>& inclusionMatrix);
    bool isLoopIncludedInLoop(const std::vector<int>& innerLoop, const std::vector<int>& outerLoop, const Plane& plane);
    void projectLoopOntoPlane(const std::vector<int>& loop, const Plane& plane, std::vector<Vector2D>& loop2D);
    bool isPointInPolygon(const Vector2D& point, const std::vector<Vector2D>& polygon);


    // Helper functions
    bool arePlanesCoplanar(const Plane& plane1, const Plane& plane2, float tolerance = 1e-5f) const;
    bool checkFaceLoopsIntersection(const Plane& plane1, const Plane& plane2, const std::vector<int>& faceLoop1, const std::vector<int>& faceLoop2, int& caseType);
    bool computePlaneIntersectionLine(const Plane& plane1, const Plane& plane2, Vector3D& pointOnLine, Vector3D& lineDirection);
bool findFaceLoopIntersections(const std::vector<int>& faceLoop1, 
                                const std::vector<int>& faceLoop2, 
                                std::vector<Vector3D>& intersectionPoints);
    void insertCuttingEdgesAndVertices(Plane& plane1, Plane& plane2, const std::vector<Vector3D>& intersectionPoints);
    void removeFaceLoop(Plane& plane, const std::vector<int>& faceLoop);

    // In wireframe.h

bool faceLoopIntersectsLine(const std::vector<Vector3D>& faceLoopVertices, const Vector3D& pointOnLine, const Vector3D& lineDirection);
bool lineIntersectsSegment(const Vector3D& linePoint, const Vector3D& lineDir, const Vector3D& segPoint1, const Vector3D& segPoint2);
bool lineCoincidesWithEdge(const std::vector<Vector3D>& faceLoopVertices, const Vector3D& pointOnLine, const Vector3D& lineDirection);
bool lineTouchesFaceLoopBoundaries(const std::vector<Vector3D>& faceLoopVertices, const Vector3D& pointOnLine, const Vector3D& lineDirection);
bool segmentsIntersect(const Vector3D& p1_start, const Vector3D& p1_end, const Vector3D& p2_start, const Vector3D& p2_end, Vector3D& intersectionPoint);
void insertVertexIntoFaceLoop(Plane& plane, const Vector3D& point, int vertexIdx);
bool pointOnSegment(const Vector3D& point, const Vector3D& segStart, const Vector3D& segEnd) const;


   Vector3D getEdgeDirection(const Edge3D& edge, const Vector3D& normal);

    bool determineFaceLoopSide(const Plane& currentPlane, const Plane& adjPlane, const Edge3D& edge, bool currentPositiveSide, bool& adjPositiveSide);
    bool isBodyLoopLegal(const std::vector<int>& S, const std::vector<FaceLoopSide>& faceLoopSides)const;
    std::vector<std::vector<int>> bodyLoops; // Declare bodyLoops

    // 3D elements
    std::vector<Vertex3D> vertices3D;
    std::vector<Edge3D> edges3D;
    std::vector<Plane> planes;

    // Body loops
    std::vector<BodyLoop> bodyLoops;          // All generated body loops
    std::vector<BodyLoop> innerBodyLoops;     // Classified inner body loops
    std::vector<BodyLoop> outerBodyLoops;     // Classified outer body loops

    // Candidate objects
    std::vector<CandidateObject> candidates;      // All possible candidates
    std::vector<CandidateObject> validCandidates; // Validated candidates matching input views

    // Helper methods for body loop classification
    bool isInnerBodyLoop(const BodyLoop& bodyLoop) const;
    bool intersectsFaceLoop(const Vector3D& origin, const Vector3D& direction, const FaceLoop& faceLoop) const;

    // Helper methods for candidate processing
    void classifyBodyLoopsInternal();
    // Other helper methods as needed
};

};

#endif // WIREFRAME_H

