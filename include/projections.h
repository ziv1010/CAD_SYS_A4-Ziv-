#ifndef PROJECTIONS_H
#define PROJECTIONS_H

#include "object3d.h"

class Projections {
public:
    // Project onto the XY (Top) plane
    static void projectToTopView(const Object3D& object, std::vector<Vertex>& projectedVertices);

    // Project onto the XZ (Front) plane
    static void projectToFrontView(const Object3D& object, std::vector<Vertex>& projectedVertices);

    // Project onto the YZ (Side) plane
    static void projectToSideView(const Object3D& object, std::vector<Vertex>& projectedVertices);
};

#endif // PROJECTIONS_H