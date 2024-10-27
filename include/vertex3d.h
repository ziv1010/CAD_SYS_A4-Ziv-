#ifndef VERTEX3D_H
#define VERTEX3D_H

#include <cstdio>

class Vertex3D {
public:
    float x, y, z;

    // Constructors
    Vertex3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Vertex3D(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    // Debug print
    void print() const {
        printf("Vertex3D(%f, %f, %f)\n", x, y, z);
    }
};

#endif // VERTEX3D_H