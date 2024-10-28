#ifndef SLICER_H
#define SLICER_H

#include "object3d.h"
#include "vertex.h"
#include "face.h"

class Slicer {
public:
    // Define a plane by point and normal
    struct Plane {
        Vertex point;  // A point on the plane
        Vertex normal; // The normal vector of the plane
    };

    // Slices the object with the plane and returns two new objects
    static void sliceObject(const Object3D& object, const Plane& plane, Object3D& objectAbove, Object3D& objectBelow);

private:
    // Helper functions
    static int classifyVertex(const Vertex& v, const Plane& plane);
    static Vertex computeIntersection(const Vertex& v1, const Vertex& v2, const Plane& plane);
};

#endif // SLICER_H
