#ifndef RENDERER_H
#define RENDERER_H

// Include OpenGL and GLFW headers
#include <GL/glew.h>   // For OpenGL types like GLuint
#include <GLFW/glfw3.h> // For GLFWwindow type

#include <vector>
#include <string>
#include "vertex.h"
#include "edge.h"
#include "face.h"
#include "object3d.h"

class Renderer
{
public:
    Renderer();
    ~Renderer();

    // Initialization and cleanup methods
    void initialize();
    void cleanup();

    // Rendering methods
    void renderModel(const Object3D& object, int modelType);
    void renderInteractive(const Object3D& object);
    void renderView(const std::vector<Edge>& edges, const std::vector<Vertex>& vertices);
    
    // Save the 3D to 2D projection as PNG
    void saveImage(const std::string& filename);

private:
    // Framebuffer and texture for rendering
    GLuint fbo;
    GLuint texture;
    std::vector<unsigned char> pixels; // For storing image data

    // Window dimensions and context
    int total_width, total_height;
    int view_width, view_height;
    GLFWwindow* window;

    // Camera and interaction controls for interactive 3D
    float camera_distance;
    float camera_angle_x;
    float camera_angle_y;
    bool left_mouse_button_pressed;
    double last_mouse_x, last_mouse_y;

    // Model data
    const Object3D* model;
    bool initialized;

    // For bounding the model in 3D space
    float model_center_x, model_center_y, model_center_z;
    float model_max_extent;

    // Static instance for GLFW callbacks
    static Renderer* instance;

    // Helper functions
    void calculateModelBounds(const Object3D& object, float& centerX, float& centerY, float& centerZ, float& maxExtent);
    void setupFramebuffer();
    void renderViews(const Object3D& object);
    void drawModel();
    void drawAxes();

    // Callback functions
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    
    // Saving the 2D projection as PNG
    void save2DProjectionAsPNG(const std::vector<Edge>& edges, const std::vector<Vertex>& vertices, const std::string& filename);
};

#endif // RENDERER_H