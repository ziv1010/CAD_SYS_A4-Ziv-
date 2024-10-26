// vertex.h

#ifndef VERTEX_H
#define VERTEX_H

class Vertex {
public:
    float x, y, z;

    // Parameterized constructor
    Vertex(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    // Default constructor
    Vertex() : x(0.0f), y(0.0f), z(0.0f) {}

    // Operator overloading for vector arithmetic
    Vertex operator-(const Vertex& other) const {
        return Vertex(x - other.x, y - other.y, z - other.z);
    }

    // Cross product
    Vertex crossProduct(const Vertex& other) const {
        return Vertex(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Setters
    void setX(float x_val) { x = x_val; }
    void setY(float y_val) { y = y_val; }
    void setZ(float z_val) { z = z_val; }

    // Getters
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

    // Dot product
    float dotProduct(const Vertex& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Addition and scalar multiplication
    Vertex operator+(const Vertex& other) const {
        return Vertex(x + other.x, y + other.y, z + other.z);
    }

    Vertex operator*(float scalar) const {
        return Vertex(x * scalar, y * scalar, z * scalar);
    }

        // Compute the length (magnitude) of the vector
    float length() const {
        return sqrt(x * x + y * y + z * z);
    }

    void normalize() {
    float length = sqrt(x * x + y * y + z * z);
    if (length > 1e-6) {
        x /= length;
        y /= length;
        z /= length;
    }
}
};

#endif // VERTEX_H