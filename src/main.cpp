#include "object3d.h"
#include "orthographic_projections.h"
#include "file_io.h"
#include <iostream>
#include "transformations.h"
#include "renderer.h"
#include "slicer.h"


int main() {
    Object3D object;

    // Read the 3D object from the input file
    read3DObjectFromFile("build/output/input3D.txt", object);

    

    // Ask the user for slicing plane parameters
    char axis;
    float position;
    std::cout << "Enter slicing axis (X, Y, Z): ";
    std::cin >> axis;
    axis = toupper(axis);
    std::cout << "Enter slicing position along axis " << axis << ": ";
    std::cin >> position;

    // Create renderer
    Renderer renderer;
    if (!renderer.initialize()) {
        return -1;
    }

    // Set slicing plane
    renderer.setSlicingPlane(axis, position);

    // Slice the object
    Object3D objectAbove, objectBelow;
    Slicer::sliceObject(object, renderer.getSlicingPlane(), objectAbove, objectBelow);

    // Add the original object to the renderer
    //renderer.addObject(object, glm::vec3(0.5f, 0.5f, 0.5f)); // Gray color

    // Add the sliced part to the renderer
    renderer.addObject(objectBelow, glm::vec3(1.0f, 0.0f, 0.0f)); // Red color

    // Run renderer
    renderer.run();


    // Project the 3D object onto 2D planes
    Projection2D topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden;
    projectToTopView(object, topViewWithoutHidden);     // Top view (XY plane)
    projectToFrontView(object, frontViewWithoutHidden); // Front view (XZ plane)
    projectToSideView(object, sideViewWithoutHidden);   // Side view (YZ plane)

    // **First, save projections without hidden line processing**
    // For projections without hidden lines, all edges are considered visible
    topViewWithoutHidden.visibleEdges = object.edges;
    frontViewWithoutHidden.visibleEdges = object.edges;
    sideViewWithoutHidden.visibleEdges = object.edges;

    // Save the projections without hidden lines
    saveCombinedProjectionAsImage("build/output/combined_views_without_hidden_lines.png",
                                  topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden);

    saveProjectionsToTextFile("build/output/projections_without_hidden_lines.txt",
                              topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden);

    std::cout << "Projections without hidden lines saved." << std::endl;

    // Now, classify edges as visible or hidden
    Projection2D topViewWithHidden = topViewWithoutHidden;
    Projection2D frontViewWithHidden = frontViewWithoutHidden;
    Projection2D sideViewWithHidden = sideViewWithoutHidden;

    classifyEdges(object, topViewWithHidden, object.faces, "XY");
    classifyEdges(object, frontViewWithHidden, object.faces, "XZ");
    classifyEdges(object, sideViewWithHidden, object.faces, "YZ");

    // Save the projections with hidden lines
    saveCombinedProjectionAsImage("build/output/combined_views_with_hidden_lines.png",
                                  topViewWithHidden, frontViewWithHidden, sideViewWithHidden);

    saveProjectionsToTextFile("build/output/projections_with_hidden_lines.txt",
                              topViewWithHidden, frontViewWithHidden, sideViewWithHidden);

    std::cout << "Projections with hidden lines saved." << std::endl;

    // Compute surface area and volume
    float surfaceArea = object.computeSurfaceArea();
    float volume = object.computeVolume();

    std::cout << "Surface Area: " << surfaceArea << std::endl;
    std::cout << "Volume: " << volume << std::endl;

    return 0;
}
