#include "graph2d_utils.h"
#include <fstream>
#include <sstream>
#include <iostream>

void readGraphsFromFile(const std::string& filename, Graph2D& topView, Graph2D& frontView, Graph2D& sideView) {
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "[Error] Cannot open input file " << filename << std::endl;
        return;
    }

    std::string line;
    Graph2D* currentGraph = nullptr;

    while (std::getline(infile, line)) {
        if (line.empty()) continue;

        if (line == "Top_View") {
            currentGraph = &topView;
            std::cout << "[Debug][Top] Reading Top View data" << std::endl;
        } else if (line == "Front_View") {
            currentGraph = &frontView;
            std::cout << "[Debug][Front] Reading Front View data" << std::endl;
        } else if (line == "Side_View") {
            currentGraph = &sideView;
            std::cout << "[Debug][Side] Reading Side View data" << std::endl;
        } else if (currentGraph) {
            static int state = 0;
            static int numVertices = 0, numEdges = 0, verticesRead = 0, edgesRead = 0;
            std::istringstream iss(line);
            if (state == 0) {
                // Reading number of vertices
                numVertices = std::stoi(line);
                verticesRead = 0;
                std::cout << "[Debug][" << currentGraph->viewName << "] Number of vertices: " << numVertices << std::endl;
                state = 1;
            } else if (state == 1 && verticesRead < numVertices) {
                // Reading vertices
                float x, y;
                iss >> x >> y;
                currentGraph->addVertex(Point2D(x, y));
                verticesRead++;
                if (verticesRead == numVertices) {
                    state = 2;
                }
            } else if (state == 2) {
                // Reading number of edges
                numEdges = std::stoi(line);
                edgesRead = 0;
                std::cout << "[Debug][" << currentGraph->viewName << "] Number of edges: " << numEdges << std::endl;
                state = 3;
            } else if (state == 3 && edgesRead < numEdges) {
                // Reading edges
                int startIdx, endIdx;
                std::string lineType;
                iss >> startIdx >> endIdx >> lineType;
                currentGraph->addEdge(Line2D(startIdx, endIdx, lineType));
                edgesRead++;
                if (edgesRead == numEdges) {
                    state = 0;
                }
            }
        } else {
            std::cerr << "[Error] Unexpected line: " << line << std::endl;
        }
    }

    infile.close();
}