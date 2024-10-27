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

    // Write probable 3D vertices to output file
    wireframe.writeVerticesToFile("output.txt");

    return 0;
}