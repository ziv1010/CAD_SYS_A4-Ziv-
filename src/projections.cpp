#include "orthographic_projections.h"
#include "stb_image_write.h"
#include <iostream>
#include <vector>
#include <cmath>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cfloat> // For FLT_MAX

void projectToXY(const Object3D& object, std::vector<std::pair<float, float>>& projectedVertices) {
    for (const auto& vertex : object.vertices) {
        projectedVertices.push_back({vertex.getX(), vertex.getY()});  // Ignore Z
    }
}

void projectToXZ(const Object3D& object, std::vector<std::pair<float, float>>& projectedVertices) {
    for (const auto& vertex : object.vertices) {
        projectedVertices.push_back({vertex.getX(), vertex.getZ()});  // Ignore Y
    }
}

void projectToYZ(const Object3D& object, std::vector<std::pair<float, float>>& projectedVertices) {
    for (const auto& vertex : object.vertices) {
        projectedVertices.push_back({vertex.getY(), vertex.getZ()});  // Ignore X
    }
}

