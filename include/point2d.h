#ifndef POINT2D_H
#define POINT2D_H

#include <cmath>
#include <cstdio>

class Point2D {
public:
    float x, y;

    // Constructors
    Point2D() : x(0.0f), y(0.0f) {}
    Point2D(float x_val, float y_val) : x(x_val), y(y_val) {}

    // Overloaded operators
    bool operator==(const Point2D& other) const {
        return (std::fabs(x - other.x) < 1e-6) && (std::fabs(y - other.y) < 1e-6);
    }

    // Debug print
    void print() const {
        printf("Point2D(%f, %f)\n", x, y);
    }
};

#endif // POINT2D_H