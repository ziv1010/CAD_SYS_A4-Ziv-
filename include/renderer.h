#ifndef RENDERER_H
#define RENDERER_H

#include "object3d.h"
#include "slicer.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

class Renderer {
public:
    Renderer();
    ~Renderer();

    bool initialize();
    void run(const Object3D& object);

    // Set initial slicing plane
    void setSlicingPlane(char axis, float position);

private:
    GLFWwindow* window;
    GLuint vao, vbo, ebo;
    GLuint shaderProgram;
    glm::mat4 modelMatrix;
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;

    float cameraZoom;

    // Original and sliced objects
    Object3D originalObject;
    Object3D slicedObject;

    // Slicing parameters
    Slicer::Plane slicingPlane;
    char slicingAxis;     // 'X', 'Y', or 'Z'
    float slicingPosition;

    // Object data for rendering
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    // Functions
    void processInput();
    void setupShaders();
    void setupBuffers();
    void updateBuffers();
    void sliceObject();

    // Input handling
    void handleSlicingInput();
};

#endif // RENDERER_H
