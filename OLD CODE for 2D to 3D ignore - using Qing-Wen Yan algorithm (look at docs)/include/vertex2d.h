#ifndef VERTEX2D_H
#define VERTEX2D_H

class Vertex2D {
public:
    float x, y;
    Vertex2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}

    // Accessors
    float getX() const { return x; }
    float getY() const { return y; }

    // Mutators
    void setX(float x) { this->x = x; }
    void setY(float y) { this->y = y; }
};

#endif // VERTEX2D_H
