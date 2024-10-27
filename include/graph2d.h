#ifndef GRAPH2D_H
#define GRAPH2D_H

#include "point2d.h"
#include "line2d.h"
#include <vector>
#include <string>

class Graph2D {
public:
    std::vector<Point2D> vertices;
    std::vector<Line2D> edges;
    std::string viewName; // "Top", "Front", or "Side"

    // Constructor
    Graph2D(const std::string& name) : viewName(name) {}

    // Methods
    void addVertex(const Point2D& point);
    void addEdge(const Line2D& line);
    void removeDuplicateVertices();
    void processIntersections();
    void handleCollinearLines();

    // Debug print
    void print() const;
};

#endif // GRAPH2D_H