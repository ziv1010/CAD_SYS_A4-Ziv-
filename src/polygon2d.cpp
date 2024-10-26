#include "polygon2d.h"

// Ray Casting algorithm to determine if point is inside polygon
bool Polygon2D::containsPoint(const Point2D& point) const {
    int intersections = 0;
    size_t n = points.size();

    for (size_t i = 0; i < n; ++i) {
        const Point2D& a = points[i];
        const Point2D& b = points[(i + 1) % n];

        // Check if the point is on an horizontal boundary
        if (fabs(a.y - b.y) < 1e-6) continue;

        // Determine if the ray intersects with the edge
        if ((point.y >= a.y && point.y < b.y) || (point.y >= b.y && point.y < a.y)) {
            float x_intersect = a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x_intersect > point.x) {
                intersections++;
            }
        }
    }

    // If the number of intersections is odd, the point is inside
    return (intersections % 2) == 1;
}