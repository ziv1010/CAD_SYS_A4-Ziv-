#ifndef _3DTO2D_H
#define _3DTO2D_H

#include "object3d.h"
#include <vector>
#include "projection2d.h"

class ThreeDToTwoD {
public:
    std::vector<Vertex> topView;
    std::vector<Vertex> frontView;
    std::vector<Vertex> sideView;

    // Project 3D object onto top (XY) plane
    void projectToTopView(const Object3D& object);

    // Project 3D object onto front (XZ) plane
    void projectToFrontView(const Object3D& object);

    // Project 3D object onto side (YZ) plane
    void projectToSideView(const Object3D& object);

    // Function to display all the views (for debugging/verification)
    void displayProjections() const;
};

#endif // _3DTO2D_H