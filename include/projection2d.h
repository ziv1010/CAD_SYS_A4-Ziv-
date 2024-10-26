// projection2d.h
#ifndef PROJECTION2D_H
#define PROJECTION2D_H

#include <vector>
#include <utility>
#include "vertex.h"
#include "edge.h"

struct Projection2D {
    std::vector<std::pair<float, float>> projectedVertices;
    std::vector<std::pair<int, int>> projectedEdges; // indices into projectedVertices

    // Members for hidden line detection
    std::vector<Edge> visibleEdges;
    std::vector<Edge> hiddenEdges;
};

#endif // PROJECTION2D_H