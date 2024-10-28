#ifndef PROJECTION2D_H
#define PROJECTION2D_H

#include <vector>
#include "vertex2d.h"
#include "edge2d.h"

class Projection2D {
public:
    std::vector<Vertex2D> vertices;
    std::vector<Edge2D> edges;
};

#endif // PROJECTION2D_H
