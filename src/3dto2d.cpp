#include "3dto2d.h"
#include <iostream>

// Project onto the top (XY) plane
void ThreeDToTwoD::projectToTopView(const Object3D& object) {
    topView.clear();
    for (const auto& vertex : object.vertices) {
        topView.emplace_back(vertex.getX(), vertex.getY(), 0);  // Z ignored
    }
}

// Project onto the front (XZ) plane
void ThreeDToTwoD::projectToFrontView(const Object3D& object) {
    frontView.clear();
    for (const auto& vertex : object.vertices) {
        frontView.emplace_back(vertex.getX(), 0, vertex.getZ());  // Y ignored
    }
}

// Project onto the side (YZ) plane
void ThreeDToTwoD::projectToSideView(const Object3D& object) {
    sideView.clear();
    for (const auto& vertex : object.vertices) {
            sideView.emplace_back(0, vertex.getY(), vertex.getZ());  // X ignored
    }
}

// Display projections for debugging/verification
void ThreeDToTwoD::displayProjections() const {
    std::cout << "Top View (XY Plane):" << std::endl;
    for (const auto& vertex : topView) {
        std::cout << "(" << vertex.getX() << ", " << vertex.getY() << ")" << std::endl;
    }

    std::cout << "Front View (XZ Plane):" << std::endl;
    for (const auto& vertex : frontView) {
        std::cout << "(" << vertex.getX() << ", " << vertex.getZ() << ")" << std::endl;
    }

    std::cout << "Side View (YZ Plane):" << std::endl;
    for (const auto& vertex : sideView) {
        std::cout << "(" << vertex.getY() << ", " << vertex.getZ() << ")" << std::endl;
    }
}