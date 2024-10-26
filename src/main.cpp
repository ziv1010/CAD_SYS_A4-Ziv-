// main.cpp

#include "object3d.h"
#include "orthographic_projections.h"
#include "file_io.h"
#include <iostream>
#include "transformations.h"

int main() {
    Object3D object;

    // Read the 3D object from the input file
    read3DObjectFromFile("build/output/input3D.txt", object);
    // Project the 3D object onto 2D planes
    Projection2D topView, frontView, sideView;
    projectToTopView(object, topView);     // Top view (XY plane)
    projectToFrontView(object, frontView); // Front view (XZ plane)
    projectToSideView(object, sideView);   // Side view (YZ plane)

    // Classify edges as visible or hidden
    classifyEdges(object, topView, object.faces, "XY");
    classifyEdges(object, frontView, object.faces, "XZ");
    classifyEdges(object, sideView, object.faces, "YZ");

    // Save the 2D projections as one combined PNG file
    saveCombinedProjectionAsImage("build/output/combined_views.png", topView, frontView, sideView);

    std::cout << "Projections with hidden lines saved to combined image." << std::endl;

        // Compute surface area and volume
    float surfaceArea = object.computeSurfaceArea();
    float volume = object.computeVolume();

    std::cout << "Surface Area: " << surfaceArea << std::endl;
    std::cout << "Volume: " << volume << std::endl;

    return 0;
}