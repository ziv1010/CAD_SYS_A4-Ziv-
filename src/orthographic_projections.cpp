#include "orthographic_projections.h"
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <iostream>


void projectToTopView(const Object3D& object, Projection2D& projection) {
    projection.projectedVertices.clear();
    projection.projectedEdges.clear();

    for (const auto& vertex : object.vertices) {
        projection.projectedVertices.push_back({vertex.getX(), vertex.getY()});
    }

    for (const auto& edge : object.edges) {
        projection.projectedEdges.push_back({edge.v1_index, edge.v2_index});
    }
}

void projectToFrontView(const Object3D& object, Projection2D& projection) {
    projection.projectedVertices.clear();
    projection.projectedEdges.clear();

    for (const auto& vertex : object.vertices) {
        projection.projectedVertices.push_back({-vertex.getZ(), vertex.getY()});  // x' = -z, y' = y
    }

    for (const auto& edge : object.edges) {
        projection.projectedEdges.push_back({edge.v1_index, edge.v2_index});
    }
}

void projectToSideView(const Object3D& object, Projection2D& projection) {
    projection.projectedVertices.clear();
    projection.projectedEdges.clear();

    for (const auto& vertex : object.vertices) {
        projection.projectedVertices.push_back({vertex.getX(), -vertex.getZ()});  // x' = x, y' = -z
    }

    for (const auto& edge : object.edges) {
        projection.projectedEdges.push_back({edge.v1_index, edge.v2_index});
    }
}

void saveProjectionsToTextFile(const std::string& filename,
                               const Projection2D& topView,
                               const Projection2D& frontView,
                               const Projection2D& sideView) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // Example format:
    // For each projection (Top, Front, Side):
    // Number of visible edges
    // List of visible edges
    // Number of hidden edges
    // List of hidden edges

    auto saveProjection = [&](const Projection2D& projection, const std::string& viewName) {
        outfile << viewName << " View" << std::endl;

        // Visible Edges
        outfile << "Visible Edges: " << projection.visibleEdges.size() << std::endl;
        for (const auto& edge : projection.visibleEdges) {
            outfile << edge.v1_index << " " << edge.v2_index << std::endl;
        }

        // Hidden Edges
        outfile << "Hidden Edges: " << projection.hiddenEdges.size() << std::endl;
        for (const auto& edge : projection.hiddenEdges) {
            outfile << edge.v1_index << " " << edge.v2_index << std::endl;
        }

        outfile << std::endl;
    };

    saveProjection(topView, "Top");
    saveProjection(frontView, "Front");
    saveProjection(sideView, "Side");

    outfile.close();
}

// Function to draw a line with optional dashed pattern
void drawLine(std::vector<unsigned char>& image, int canvasWidth, int canvasHeight,
             int x0, int y0, int x1, int y1, bool dashed = false) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    int dashLength = 5;
    int dashCount = 0;
    bool draw = true;

    while (true) {
        if (x0 >= 0 && x0 < canvasWidth && y0 >= 0 && y0 < canvasHeight) {
            int index = (y0 * canvasWidth + x0) * 3;
            if (draw) {
                image[index] = 0;     // Red
                image[index + 1] = 0; // Green
                image[index + 2] = 0; // Blue
            } else if (dashed) {
                // Optionally, set a different color or skip drawing
                // For dashed lines, we skip drawing the pixel
            }
        }

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }

        if (dashed) {
            dashCount++;
            if (dashCount >= dashLength) {
                dashCount = 0;
                draw = !draw;
            }
        }
    }
}

// Function to scale and draw projections onto one combined PNG
void saveCombinedProjectionAsImage(const std::string& filename,
                                   const Projection2D& topView,
                                   const Projection2D& frontView,
                                   const Projection2D& sideView) {
    const int singleViewWidth = 800;
    const int singleViewHeight = 800;
    const int canvasWidth = singleViewWidth * 3;  // One section for each view
    const int canvasHeight = singleViewHeight;

    // Create a blank white image
    std::vector<unsigned char> image(canvasWidth * canvasHeight * 3, 255); // White background

    auto drawProjection = [&](const Projection2D& projection, int offsetX, int offsetY) {
        // Find the min and max coordinates for scaling
        float minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;

        for (const auto& vertex : projection.projectedVertices) {
            minX = std::min(minX, vertex.first);
            minY = std::min(minY, vertex.second);
            maxX = std::max(maxX, vertex.first);
            maxY = std::max(maxY, vertex.second);
        }

        // Add a small margin for padding
        float margin = 0.05f * std::max(maxX - minX, maxY - minY);
        minX -= margin;
        minY -= margin;
        maxX += margin;
        maxY += margin;

        // Precompute scaling factors
        float scaleX = (singleViewWidth - 1) / (maxX - minX);
        float scaleY = (singleViewHeight - 1) / (maxY - minY);

        // Store image coordinates for vertices
        std::vector<std::pair<int, int>> imageCoords(projection.projectedVertices.size());

        for (size_t i = 0; i < projection.projectedVertices.size(); ++i) {
            float x = projection.projectedVertices[i].first;
            float y = projection.projectedVertices[i].second;

            int imgX = static_cast<int>((x - minX) * scaleX) + offsetX;
            int imgY = static_cast<int>((y - minY) * scaleY) + offsetY;

            // Invert y to match image coordinate system
            imgY = singleViewHeight - 1 - imgY + offsetY;

            imageCoords[i] = {imgX, imgY};

            // Draw the vertex as a small black dot
            if (imgX >= 0 && imgX < canvasWidth && imgY >= 0 && imgY < canvasHeight) {
                int index = (imgY * canvasWidth + imgX) * 3;
                image[index] = 0;     // Red
                image[index + 1] = 0; // Green
                image[index + 2] = 0; // Blue
            }
        }

        // Draw visible edges as solid lines
        for (const auto& edge : projection.visibleEdges) {
            int v1 = edge.v1_index;
            int v2 = edge.v2_index;
            int x0 = imageCoords[v1].first;
            int y0 = imageCoords[v1].second;
            int x1 = imageCoords[v2].first;
            int y1 = imageCoords[v2].second;

            drawLine(image, canvasWidth, canvasHeight, x0, y0, x1, y1, false);
        }

        // Draw hidden edges as dashed lines
        for (const auto& edge : projection.hiddenEdges) {
            int v1 = edge.v1_index;
            int v2 = edge.v2_index;
            int x0 = imageCoords[v1].first;
            int y0 = imageCoords[v1].second;
            int x1 = imageCoords[v2].first;
            int y1 = imageCoords[v2].second;

            drawLine(image, canvasWidth, canvasHeight, x0, y0, x1, y1, true);
        }
    };

    // Draw top view in the first section
    drawProjection(topView, 0, 0);

    // Draw front view in the second section
    drawProjection(frontView, singleViewWidth, 0);

    // Draw side view in the third section
    drawProjection(sideView, singleViewWidth * 2, 0);

    // Save the image using stb_image_write
    if (stbi_write_png(filename.c_str(), canvasWidth, canvasHeight, 3, image.data(), canvasWidth * 3)) {
        std::cout << "Saved combined image to " << filename << std::endl;
    } else {
        std::cerr << "Error: Failed to save combined image to " << filename << std::endl;
    }
}

// Helper function to compute the midpoint of an edge in 2D
Point2D computeMidpoint2D(const Projection2D& projection, const Edge& edge) {
    float x0 = projection.projectedVertices[edge.v1_index].first;
    float y0 = projection.projectedVertices[edge.v1_index].second;
    float x1 = projection.projectedVertices[edge.v2_index].first;
    float y1 = projection.projectedVertices[edge.v2_index].second;
    return Point2D{(x0 + x1) / 2.0f, (y0 + y1) / 2.0f};
}

// Helper function to compute the 3D midpoint corresponding to a 2D point
Vertex compute3DMidpoint(const Object3D& object, const Edge& edge, const Point2D& midpoint2D, const std::string& plane) {
    const Vertex& v1 = object.vertices[edge.v1_index];
    const Vertex& v2 = object.vertices[edge.v2_index];

    // Calculate parameter t for interpolation
    float t = 0.0f;
    if (plane == "XY") {
        float dx = v2.getX() - v1.getX();
        float dy = v2.getY() - v1.getY();
        if (fabs(dx) > fabs(dy)) {
            t = (midpoint2D.x - v1.getX()) / dx;
        } else {
            t = (midpoint2D.y - v1.getY()) / dy;
        }
    } else if (plane == "XZ") {
        float dx = v2.getX() - v1.getX();
        float dz = v2.getZ() - v1.getZ();
        if (fabs(dx) > fabs(dz)) {
            t = (midpoint2D.x - v1.getX()) / dx;
        } else {
            t = (midpoint2D.y - v1.getZ()) / dz;
        }
    } else { // "YZ"
        float dy = v2.getY() - v1.getY();
        float dz = v2.getZ() - v1.getZ();
        if (fabs(dy) > fabs(dz)) {
            t = (midpoint2D.x - v1.getY()) / dy;
        } else {
            t = (midpoint2D.y - v1.getZ()) / dz;
        }
    }

    // Clamp t to [0,1] to avoid numerical issues
    t = std::max(0.0f, std::min(1.0f, t));

    // Compute the 3D midpoint
    float x = v1.getX() + t * (v2.getX() - v1.getX());
    float y = v1.getY() + t * (v2.getY() - v1.getY());
    float z = v1.getZ() + t * (v2.getZ() - v1.getZ());

    return Vertex(x, y, z);
}

void classifyEdges(const Object3D& object,
                   Projection2D& projection,
                   const std::vector<Face>& faces,
                   const std::string& plane) {
    projection.visibleEdges.clear();
    projection.hiddenEdges.clear();

    // Precompute face normals and plane equations
    // Precompute face normals and plane equations
struct FacePlane {
    float A, B, C, D; // Plane equation coefficients
    Polygon2D projectedFace; // 2D projection of the face
};

std::vector<FacePlane> facePlanes;

for (const auto& face : faces) {
    // Get vertex indices of the face
    const std::vector<int>& indices = face.getVertexIndices();
    if (indices.size() < 3) continue; // Skip degenerate faces

    // Get the vertices of the face
    std::vector<Vertex> faceVertices;
    for (int idx : indices) {
        faceVertices.push_back(object.vertices[idx]);
    }

    // Compute normal vector and plane equation
    const Vertex& p0 = faceVertices[0];
    const Vertex& p1 = faceVertices[1];
    const Vertex& p2 = faceVertices[2];

    Vertex u = p1 - p0;
    Vertex v = p2 - p0;
    Vertex n = u.crossProduct(v);

    float A = n.getX();
    float B = n.getY();
    float C = n.getZ();
    float D = - (A * p0.getX() + B * p0.getY() + C * p0.getZ());

    // Skip faces with zero normal vector
    if (fabs(A) < 1e-6 && fabs(B) < 1e-6 && fabs(C) < 1e-6) continue;

    // Project the face onto the plane
    Polygon2D projectedFace;
    for (int idx : indices) {
        const Vertex& vtx = object.vertices[idx];
        if (plane == "XY") {
            projectedFace.points.push_back(Point2D{vtx.getX(), vtx.getY()});
        } else if (plane == "XZ") {
            projectedFace.points.push_back(Point2D{vtx.getX(), vtx.getZ()});
        } else { // "YZ"
            projectedFace.points.push_back(Point2D{vtx.getY(), vtx.getZ()});
        }
    }

    facePlanes.push_back({A, B, C, D, projectedFace});
    }

    // Classify edges
    for (const auto& edge : object.edges) {
        // Project the edge's vertices
        auto proj_v1 = projection.projectedVertices[edge.v1_index];
        auto proj_v2 = projection.projectedVertices[edge.v2_index];

        // Compute the 2D midpoint
        float mid_x = (proj_v1.first + proj_v2.first) / 2.0f;
        float mid_y = (proj_v1.second + proj_v2.second) / 2.0f;

        // Compute the 3D midpoint
        const Vertex& v1 = object.vertices[edge.v1_index];
        const Vertex& v2 = object.vertices[edge.v2_index];
        Vertex mid3D((v1.getX() + v2.getX()) / 2.0f,
                     (v1.getY() + v2.getY()) / 2.0f,
                     (v1.getZ() + v2.getZ()) / 2.0f);

        bool isHidden = false;

        // Check against each face
        for (const auto& facePlane : facePlanes) {
            // Check if the 2D midpoint lies within the projected face
            Point2D midpoint2D = {mid_x, mid_y};
            if (facePlane.projectedFace.containsPoint(midpoint2D)) {
                // Compute depth of face at the midpoint
                float A = facePlane.A;
                float B = facePlane.B;
                float C = facePlane.C;
                float D = facePlane.D;

                if (plane == "XY") {
                    if (fabs(C) < 1e-6) continue;
                    float faceZ = - (A * mid3D.getX() + B * mid3D.getY() + D) / C;
                    if (faceZ > mid3D.getZ()) {
                        isHidden = true;
                        break;
                    }
                } else if (plane == "XZ") {
                    if (fabs(B) < 1e-6) continue;
                    float faceY = - (A * mid3D.getX() + C * mid3D.getZ() + D) / B;
                    if (faceY > mid3D.getY()) {
                        isHidden = true;
                        break;
                    }
                } else if (plane == "YZ") {
                    if (fabs(A) < 1e-6) continue;
                    float faceX = - (B * mid3D.getY() + C * mid3D.getZ() + D) / A;
                    if (faceX > mid3D.getX()) {
                        isHidden = true;
                        break;
                    }
                }
            }
        }

        if (isHidden) {
            projection.hiddenEdges.push_back(edge);
        } else {
            projection.visibleEdges.push_back(edge);
        }
    }
}