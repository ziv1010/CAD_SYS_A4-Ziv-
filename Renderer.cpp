#include "Renderer.h"
#include <iostream>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cfloat> // For FLT_MAX
#include <algorithm> // For std::min and std::max

// Initialize the static instance pointer
Renderer* Renderer::instance = nullptr;

Renderer::Renderer()
{
    fbo = 0;
    texture = 0;
    total_width = 0;
    total_height = 0;
    view_width = 800;
    view_height = 800;
    window = nullptr;
    initialized = false;

    // Initialize camera and interaction parameters
    camera_distance = 5.0f;
    camera_angle_x = 0.0f;
    camera_angle_y = 0.0f;
    left_mouse_button_pressed = false;
    last_mouse_x = 0.0;
    last_mouse_y = 0.0;
    model = nullptr;
}

Renderer::~Renderer()
{
    cleanup();
}

void Renderer::renderModel(const Object3D& object, int modelType)
{
    if (modelType == 3)
    {
        // Perform 3D to 2D projections and save them as PNG
        renderViews(object);
    }
    else if (modelType == 2)
    {
        // Render the 3D model interactively
        renderInteractive(object);
    }
    else
    {
        std::cerr << "Invalid modelType provided to Renderer.\n";
    }
}

void Renderer::renderViews(const Object3D& object)
{
    // Create separate vectors for the three 2D projections
    std::vector<Vertex> topViewVertices, frontViewVertices, sideViewVertices;

    // Project onto XY plane (Top View)
    for (const auto& vertex : object.vertices) {
        topViewVertices.emplace_back(vertex.getX(), vertex.getY(), 0);  // Ignore Z coordinate
    }

    // Project onto XZ plane (Front View)
    for (const auto& vertex : object.vertices) {
        frontViewVertices.emplace_back(vertex.getX(), 0, vertex.getZ());  // Ignore Y coordinate
    }

    // Project onto YZ plane (Side View)
    for (const auto& vertex : object.vertices) {
        sideViewVertices.emplace_back(0, vertex.getY(), vertex.getZ());  // Ignore X coordinate
    }

    // Save each view as PNG
    save2DProjectionAsPNG(object.edges, topViewVertices, "top_view.png");
    save2DProjectionAsPNG(object.edges, frontViewVertices, "front_view.png");
    save2DProjectionAsPNG(object.edges, sideViewVertices, "side_view.png");
}

void Renderer::save2DProjectionAsPNG(const std::vector<Edge>& edges, const std::vector<Vertex>& vertices, const std::string& filename)
{
    const int width = 800;
    const int height = 800;

    std::vector<unsigned char> image(width * height * 3, 255); // White background

    float max_coord = 1.0f;
    for (const auto& vertex : vertices)
    {
        max_coord = std::max(max_coord, std::abs(vertex.getX()));
        max_coord = std::max(max_coord, std::abs(vertex.getY()));  // Only use 2D (X, Y)
    }
    max_coord *= 1.1f; // Add a margin for scaling

    for (const auto& edge : edges)
    {
        // Map 2D coordinates to the image space
        int x1 = static_cast<int>(((vertices[edge.v1_index].getX() / max_coord) + 1.0f) * 0.5f * width);
        int y1 = static_cast<int>(((vertices[edge.v1_index].getY() / max_coord) + 1.0f) * 0.5f * height);
        int x2 = static_cast<int>(((vertices[edge.v2_index].getX() / max_coord) + 1.0f) * 0.5f * width);
        int y2 = static_cast<int>(((vertices[edge.v2_index].getY() / max_coord) + 1.0f) * 0.5f * height);

        // Draw the edge as a line on the image
        int dx = abs(x2 - x1), dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height)
            {
                int index = (y1 * width + x1) * 3;
                image[index] = 0;   // Red
                image[index + 1] = 0; // Green
                image[index + 2] = 0; // Blue
            }

            if (x1 == x2 && y1 == y2)
                break;

            int e2 = err * 2;
            if (e2 > -dy)
            {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y1 += sy;
            }
        }
    }

    // Save the image using stb_image_write
    if (stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3))
    {
        std::cout << "Saved " << filename << "\n";
    }
    else
    {
        std::cerr << "Failed to save image\n";
    }
}

void Renderer::cleanup()
{
    if (texture != 0)
    {
        glDeleteTextures(1, &texture);
        texture = 0;
    }
    if (fbo != 0)
    {
        glDeleteFramebuffers(1, &fbo);
        fbo = 0;
    }
    if (window != nullptr)
    {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
    initialized = false;
}

void Renderer::renderInteractive(const Object3D& object)
{
    this->model = &object;

    // Calculate model bounds (center and max extent for proper rendering)
    float centerX, centerY, centerZ, maxExtent;
    calculateModelBounds(object, centerX, centerY, centerZ, maxExtent);

    model_center_x = centerX;
    model_center_y = centerY;
    model_center_z = centerZ;
    model_max_extent = maxExtent;

    camera_distance = maxExtent * 1.5f;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
        glClearColor(0.9f, 0.9f, 0.9f, 1.0f); // Light gray background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set projection matrix
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspect_ratio = static_cast<float>(view_width) / static_cast<float>(view_height);
        gluPerspective(45.0, aspect_ratio, 0.1, model_max_extent * 20.0f);

        // Set modelview matrix
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Apply camera transformations
        glTranslatef(0.0f, 0.0f, -camera_distance);
        glRotatef(camera_angle_x, 1.0f, 0.0f, 0.0f);
        glRotatef(camera_angle_y, 0.0f, 1.0f, 0.0f);

        // Translate model to center it
        glTranslatef(-model_center_x, -model_center_y, -model_center_z);

        // Draw the model
        drawModel();

        glfwSwapBuffers(window);
    }

    cleanup();
}

void Renderer::calculateModelBounds(const Object3D& object, float& centerX, float& centerY, float& centerZ, float& maxExtent)
{
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;

    for (const auto& vertex : object.vertices)
    {
        minX = std::min(minX, vertex.getX());
        minY = std::min(minY, vertex.getY());
        minZ = std::min(minZ, vertex.getZ());

        maxX = std::max(maxX, vertex.getX());
        maxY = std::max(maxY, vertex.getY());
        maxZ = std::max(maxZ, vertex.getZ());
    }

    centerX = (minX + maxX) / 2.0f;
    centerY = (minY + maxY) / 2.0f;
    centerZ = (minZ + maxZ) / 2.0f;

    float extentX = maxX - minX;
    float extentY = maxY - minY;
    float extentZ = maxZ - minZ;

    maxExtent = std::max({extentX, extentY, extentZ});
}

void Renderer::drawModel()
{
    if (!model)
        return;

    // Set the color for edges
    glColor3f(0.0f, 0.0f, 0.0f); // Black color for edges
    glLineWidth(1.0f);

    glBegin(GL_LINES);
    for (const auto& edge : model->edges)
    {
        glVertex3f(edge.v1.getX(), edge.v1.getY(), edge.v1.getZ());
        glVertex3f(edge.v2.getX(), edge.v2.getY(), edge.v2.getZ());
    }
    glEnd();
}