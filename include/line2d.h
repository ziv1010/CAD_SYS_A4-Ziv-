#ifndef LINE2D_H
#define LINE2D_H

#include "point2d.h"
#include <string>
#include <cstdio>

class Line2D {
public:
    int startIdx;
    int endIdx;
    std::string type; // "solid" or "dashed"

    // Constructors
    Line2D() : startIdx(0), endIdx(0), type("solid") {}
    Line2D(int s, int e, const std::string& t = "solid")
        : startIdx(s), endIdx(e), type(t) {}

    // Debug print
    void print() const {
        printf("Line2D from %d to %d [%s]\n",
               startIdx, endIdx, type.c_str());
    }
};

#endif // LINE2D_H