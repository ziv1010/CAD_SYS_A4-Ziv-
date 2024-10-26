#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "object3d.h"

class Transformations {
public:
    // Rotation around X, Y, Z axes
    static void rotateX(Object3D& object, float angle);
    static void rotateY(Object3D& object, float angle);
    static void rotateZ(Object3D& object, float angle);

    // Translation along X, Y, Z axes
    static void translate(Object3D& object, float tx, float ty, float tz);

    // Uniform Scaling
    static void scale(Object3D& object, float scaleFactor);
};

#endif // TRANSFORMATIONS_H