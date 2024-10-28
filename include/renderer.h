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
    void run();

    // Set initial slicing plane
    void setSlicingPlane(char axis, float position);
    void addObject(const Object3D& object, const glm::vec3& color);
    const Slicer::Plane& getSlicingPlane() const { return slicingPlane; }
private:
    GLFWwindow* window;
    GLuint vao, vbo, ebo;
    GLuint shaderProgram;
    glm::mat4 modelMatrix;
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;

    float cameraZoom;
    float rotationAngle; // For rotation

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

    struct RenderObject {
    Object3D object;
    glm::vec3 color;
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
};
 std::vector<RenderObject> renderObjects;
 
};

#endif // RENDERER_H
