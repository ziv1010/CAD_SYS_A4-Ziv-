#ifndef GRAPH2D_UTILS_H
#define GRAPH2D_UTILS_H

#include "graph2d.h"
#include <string>

void readGraphsFromFile(const std::string& filename, Graph2D& topView, Graph2D& frontView, Graph2D& sideView);

#endif // GRAPH2D_UTILS_H