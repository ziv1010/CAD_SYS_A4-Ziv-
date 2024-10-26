#ifndef POLYGON2D_H
#define POLYGON2D_H

#include <vector>

struct Point2D {
    float x, y;
};

class Polygon2D {
public:
    std::vector<Point2D> points;

    // Check if a point is inside the polygon using the ray casting algorithm
    bool containsPoint(const Point2D& point) const;
};

#endif // POLYGON2D_H