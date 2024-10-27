#include "graph2d.h"
#include "graph2d_utils.h"
#include "wireframe_model.h"
#include <iostream>

int main() {
    Graph2D topView("Top");
    Graph2D frontView("Front");
    Graph2D sideView("Side");

    // Read input file
    readGraphsFromFile("build/output/input2d.txt", topView, frontView, sideView);

    // Process each view
    topView.removeDuplicateVertices();
    topView.processIntersections();
    topView.handleCollinearLines();

    frontView.removeDuplicateVertices();
    frontView.processIntersections();
    frontView.handleCollinearLines();

    sideView.removeDuplicateVertices();
    sideView.processIntersections();
    sideView.handleCollinearLines();

    // Generate probable 3D vertices
    WireframeModel wireframe;
    wireframe.generateProbableVertices(topView, frontView, sideView);

    // Generate probable 3D edges
    wireframe.generateProbableEdges(topView, frontView, sideView);

    // Validate vertices and edges
    wireframe.validateVerticesAndEdges();

    // Generate probable faces
    wireframe.generateProbableFaces();

    // Write wireframe model to output file
    wireframe.writeToFile("output.txt");

    // Optionally, print the wireframe model
    wireframe.print();

    return 0;
}
