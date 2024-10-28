// main.cpp

#include "object3d.h"
#include "orthographic_projections.h"
#include "file_io.h"
#include <iostream>
#include "transformations.h"
#include "renderer.h"
#include "slicer.h"

int main() {
    // Read the 3D object from the input file
    Object3D originalObject;
    read3DObjectFromFile("build/output/input3D.txt", originalObject);

    Object3D object = originalObject; // Working copy of the object

    bool exitProgram = false;

    while (!exitProgram) {
        // Display menu to user
        std::cout << "\nSelect an operation:" << std::endl;
        std::cout << "1. Apply Transformations (Rotate, Translate, Scale)" << std::endl;
        std::cout << "2. Perform Slicing" << std::endl;
        std::cout << "3. Perform Orthographic Projections and Calculate Surface Area and Volume" << std::endl;
        std::cout << "4. Reset Object to Original State" << std::endl;
        std::cout << "5. Exit" << std::endl;
        int choice;
        std::cin >> choice;

        switch (choice) {
            case 1: {
                // Apply transformations
                bool doneTransformations = false;
                while (!doneTransformations) {
                    std::cout << "\nSelect a transformation to apply:" << std::endl;
                    std::cout << "1. Rotate" << std::endl;
                    std::cout << "2. Translate" << std::endl;
                    std::cout << "3. Scale" << std::endl;
                    std::cout << "4. Done applying transformations" << std::endl;
                    int transformChoice;
                    std::cin >> transformChoice;

                    switch (transformChoice) {
                        case 1: {
                            // Rotate
                            char axis;
                            float angle;
                            std::cout << "Enter axis to rotate around (X, Y, Z): ";
                            std::cin >> axis;
                            axis = toupper(axis);
                            std::cout << "Enter rotation angle in degrees: ";
                            std::cin >> angle;
                            if (axis == 'X') {
                                Transformations::rotateX(object, angle);
                            } else if (axis == 'Y') {
                                Transformations::rotateY(object, angle);
                            } else if (axis == 'Z') {
                                Transformations::rotateZ(object, angle);
                            } else {
                                std::cout << "Invalid axis." << std::endl;
                            }
                            break;
                        }
                        case 2: {
                            // Translate
                            float tx, ty, tz;
                            std::cout << "Enter translation along X axis: ";
                            std::cin >> tx;
                            std::cout << "Enter translation along Y axis: ";
                            std::cin >> ty;
                            std::cout << "Enter translation along Z axis: ";
                            std::cin >> tz;
                            Transformations::translate(object, tx, ty, tz);
                            break;
                        }
                        case 3: {
                            // Scale
                            float scaleFactor;
                            std::cout << "Enter scaling factor: ";
                            std::cin >> scaleFactor;
                            Transformations::scale(object, scaleFactor);
                            break;
                        }
                        case 4:
                            // Done applying transformations
                            doneTransformations = true;
                            break;
                        default:
                            std::cout << "Invalid choice. Please select again." << std::endl;
                    }
                }

                // Render the transformed object
                Renderer renderer;
                if (!renderer.initialize()) {
                    return -1;
                }
                // Add the transformed object to the renderer
                renderer.addObject(object, glm::vec3(0.5f, 0.5f, 0.5f)); // Gray color
                renderer.run();

                // After window is closed, perform orthographic projections and calculate surface area and volume

                // Project the 3D object onto 2D planes
                Projection2D topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden;
                projectToTopView(object, topViewWithoutHidden);     // Top view (XY plane)
                projectToFrontView(object, frontViewWithoutHidden); // Front view (XZ plane)
                projectToSideView(object, sideViewWithoutHidden);   // Side view (YZ plane)

                // Save projections without hidden lines
                topViewWithoutHidden.visibleEdges = object.edges;
                frontViewWithoutHidden.visibleEdges = object.edges;
                sideViewWithoutHidden.visibleEdges = object.edges;

                saveCombinedProjectionAsImage("build/output/combined_views_without_hidden_lines.png",
                                              topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden);

                saveProjectionsToTextFile("build/output/projections_without_hidden_lines.txt",
                                          topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden);

                std::cout << "Projections without hidden lines saved." << std::endl;

                // Classify edges as visible or hidden
                Projection2D topViewWithHidden = topViewWithoutHidden;
                Projection2D frontViewWithHidden = frontViewWithoutHidden;
                Projection2D sideViewWithHidden = sideViewWithoutHidden;

                classifyEdges(object, topViewWithHidden, object.faces, "XY");
                classifyEdges(object, frontViewWithHidden, object.faces, "XZ");
                classifyEdges(object, sideViewWithHidden, object.faces, "YZ");

                // Save projections with hidden lines
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

                break;
            }
            case 2: {
                // Perform slicing
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
                renderer.addObject(object, glm::vec3(0.5f, 0.5f, 0.5f)); // Gray color

                // Add the sliced part to the renderer
                renderer.addObject(objectBelow, glm::vec3(1.0f, 0.0f, 0.0f)); // Red color

                // Run renderer
                renderer.run();

                break;
            }
            case 3: {
                // Perform orthographic projections and calculate surface area and volume

                // Project the 3D object onto 2D planes
                Projection2D topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden;
                projectToTopView(object, topViewWithoutHidden);     // Top view (XY plane)
                projectToFrontView(object, frontViewWithoutHidden); // Front view (XZ plane)
                projectToSideView(object, sideViewWithoutHidden);   // Side view (YZ plane)

                // Save projections without hidden lines
                topViewWithoutHidden.visibleEdges = object.edges;
                frontViewWithoutHidden.visibleEdges = object.edges;
                sideViewWithoutHidden.visibleEdges = object.edges;

                saveCombinedProjectionAsImage("build/output/combined_views_without_hidden_lines.png",
                                              topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden);

                saveProjectionsToTextFile("build/output/projections_without_hidden_lines.txt",
                                          topViewWithoutHidden, frontViewWithoutHidden, sideViewWithoutHidden);

                std::cout << "Projections without hidden lines saved." << std::endl;

                // Classify edges as visible or hidden
                Projection2D topViewWithHidden = topViewWithoutHidden;
                Projection2D frontViewWithHidden = frontViewWithoutHidden;
                Projection2D sideViewWithHidden = sideViewWithoutHidden;

                classifyEdges(object, topViewWithHidden, object.faces, "XY");
                classifyEdges(object, frontViewWithHidden, object.faces, "XZ");
                classifyEdges(object, sideViewWithHidden, object.faces, "YZ");

                // Save projections with hidden lines
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

                break;
            }
            case 4: {
                // Reset object to original state
                object = originalObject;
                std::cout << "Object reset to original state." << std::endl;
                break;
            }
            case 5:
                // Exit
                exitProgram = true;
                break;
            default:
                std::cout << "Invalid choice. Please select again." << std::endl;
        }
    }

    return 0;
}
