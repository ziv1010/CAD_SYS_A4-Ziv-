// vector3d.h

#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <cmath>

class Vector3D {
public:
    float x, y, z;

    Vector3D(float x_=0.0f, float y_=0.0f, float z_=0.0f)
        : x(x_), y(y_), z(z_) {}

    // Cross product
    Vector3D cross(const Vector3D& v) const {
        return Vector3D(y * v.z - z * v.y,
                        z * v.x - x * v.z,
                        x * v.y - y * v.x);
    }

        // Dot product
    float dot(const Vector3D& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Length of the vector
    float length() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    // Normalize the vector
    Vector3D normalize() const {
        float len = length();
        if (len > 1e-6) {
            return Vector3D(x / len, y / len, z / len);
        }
        return Vector3D(0.0f, 0.0f, 0.0f); // Zero vector if length is too small
    }

    // Operator Overloads

    // Member function for Vector3D + Vector3D
    Vector3D operator+(const Vector3D& other) const {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    // Member function for Vector3D - Vector3D
    Vector3D operator-(const Vector3D& other) const {
        return Vector3D(x - other.x, y - other.y, z - other.z);
    }

    // Member function for Vector3D * float
    Vector3D operator*(float scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }

    // Friend function for float * Vector3D
    friend Vector3D operator*(float scalar, const Vector3D& vec) {
        return Vector3D(vec.x * scalar, vec.y * scalar, vec.z * scalar);
    }

};

#endif // VECTOR3D_H
