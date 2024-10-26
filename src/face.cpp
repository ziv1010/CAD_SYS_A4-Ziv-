#include "face.h"
#include <string>

Polygon2D Face::projectFaceToPlane(const std::string& plane, const std::vector<Vertex>& vertices) const {
    Polygon2D polygon;

    for (int idx : vertexIndices) {
        const Vertex& v = vertices[idx];
        if (plane == "XY") {
            polygon.points.push_back(Point2D{v.getX(), v.getY()});
        } else if (plane == "XZ") {
            polygon.points.push_back(Point2D{v.getX(), v.getZ()});
        } else if (plane == "YZ") {
            polygon.points.push_back(Point2D{v.getY(), v.getZ()});
        }
    }

    return polygon;
}