#include "renderer.h"
#include <iostream>
#include <vector>
#include <map>
#include <cctype>

// Include GLM headers for transformations
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Shader sources
const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

void main()
{
    FragColor = vec4(0.7, 0.7, 0.7, 1.0);
}
)";

// Rest of your Renderer class implementation...

Renderer::Renderer()
    : window(nullptr), vao(0), vbo(0), ebo(0), shaderProgram(0),
      cameraZoom(45.0f), slicingAxis('X'), slicingPosition(0.0f)
{
}

Renderer::~Renderer()
{
    // Cleanup
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
    glfwTerminate();
}

bool Renderer::initialize()
{
    // Initialization code remains the same...
     // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

     // **Add these lines to request OpenGL 3.3 core profile**
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // OpenGL version 3.x
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); // OpenGL version x.3
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Use core profile

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on macOS
#endif

    // Create window
    window = glfwCreateWindow(800, 600, "3D Object Renderer", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);

    // Pass pointer to this Renderer to callbacks
    glfwSetWindowUserPointer(window, this);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW:" << glewGetErrorString(err) << std::endl;
        return false;
    }

    // Configure OpenGL settings
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    
    // Set viewport size callback
    glfwSetFramebufferSizeCallback(window, [](GLFWwindow*, int width, int height) {
        glViewport(0, 0, width, height);
    });

    // Scroll callback for zooming
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        renderer->cameraZoom -= (float)yoffset;
        if (renderer->cameraZoom < 1.0f)
            renderer->cameraZoom = 1.0f;
        if (renderer->cameraZoom > 45.0f)
            renderer->cameraZoom = 45.0f;
        renderer->projectionMatrix = glm::perspective(glm::radians(renderer->cameraZoom), 800.0f / 600.0f, 0.1f, 100.0f);
    });
    // Set key callback for input handling
    glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            renderer->handleSlicingInput();
        }
    });

    return true;
}

void Renderer::setSlicingPlane(char axis, float position)
{
    slicingAxis = toupper(axis);
    slicingPosition = position;
    // Initialize slicing plane
    if (slicingAxis == 'X') {
        slicingPlane.point = Vertex(slicingPosition, 0.0f, 0.0f);
        slicingPlane.normal = Vertex(1.0f, 0.0f, 0.0f);
    } else if (slicingAxis == 'Y') {
        slicingPlane.point = Vertex(0.0f, slicingPosition, 0.0f);
        slicingPlane.normal = Vertex(0.0f, 1.0f, 0.0f);
    } else if (slicingAxis == 'Z') {
        slicingPlane.point = Vertex(0.0f, 0.0f, slicingPosition);
        slicingPlane.normal = Vertex(0.0f, 0.0f, 1.0f);
    }
}

void Renderer::run(const Object3D& object)
{
    // Store original object
    originalObject = object;

    setupShaders();
    sliceObject(); // Initial slicing
    setupBuffers();

    // Set up camera
    viewMatrix = glm::lookAt(glm::vec3(0.0f, 0.0f, 5.0f),
                             glm::vec3(0.0f, 0.0f, 0.0f),
                             glm::vec3(0.0f, 1.0f, 0.0f));
    projectionMatrix = glm::perspective(glm::radians(cameraZoom), 800.0f / 600.0f, 0.1f, 100.0f);

    // No rotation
    modelMatrix = glm::mat4(1.0f);

    // Main loop
    while (!glfwWindowShouldClose(window)) {
    processInput();

    // Clear the screen and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update projection matrix in case cameraZoom changed
    projectionMatrix = glm::perspective(glm::radians(cameraZoom), 800.0f / 600.0f, 0.1f, 100.0f);

    // Use shader
    glUseProgram(shaderProgram);

    // Set uniforms
    GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));

    // Draw object
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // Swap buffers
    glfwSwapBuffers(window);
    glfwPollEvents();
}

}

void Renderer::processInput()
{
    // Close window on ESC
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // Handle slicing input
    handleSlicingInput();
}

void Renderer::handleSlicingInput()
{
    // Check for axis change
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        slicingAxis = 'X';
    }
    if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) {
        slicingAxis = 'Y';
    }
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        slicingAxis = 'Z';
    }

    // Adjust slicing position
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        slicingPosition += 0.01f; // Adjust increment as needed
        sliceObject();
        updateBuffers();
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        slicingPosition -= 0.01f; // Adjust increment as needed
        sliceObject();
        updateBuffers();
    }

    // Update slicing plane
    if (slicingAxis == 'X') {
        slicingPlane.point = Vertex(slicingPosition, 0.0f, 0.0f);
        slicingPlane.normal = Vertex(1.0f, 0.0f, 0.0f);
    } else if (slicingAxis == 'Y') {
        slicingPlane.point = Vertex(0.0f, slicingPosition, 0.0f);
        slicingPlane.normal = Vertex(0.0f, 1.0f, 0.0f);
    } else if (slicingAxis == 'Z') {
        slicingPlane.point = Vertex(0.0f, 0.0f, slicingPosition);
        slicingPlane.normal = Vertex(0.0f, 0.0f, 1.0f);
    }
}

void Renderer::sliceObject()
{
    // Slice the original object
    Object3D objectAbove, objectBelow;
    Slicer::sliceObject(originalObject, slicingPlane, objectAbove, objectBelow);

    // For rendering, we'll display the part below the slicing plane
    slicedObject = objectBelow;
}

void Renderer::setupShaders()
{
// Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // Check vertex shader
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "Error: Vertex shader compilation failed:\n" << infoLog << std::endl;
    }

    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // Check fragment shader
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "Error: Fragment shader compilation failed:\n" << infoLog << std::endl;
    }

    // Link shaders into program
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // Check program
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "Error: Shader program linking failed:\n" << infoLog << std::endl;
    }

    // Delete shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Renderer::setupBuffers()
{
    // Convert sliced object data to OpenGL buffers
    vertices.clear();
    indices.clear();

    // Map from Object3D vertex indices to OpenGL indices
    std::map<int, unsigned int> vertexIndexMap;

    unsigned int currentIndex = 0;
    for (size_t i = 0; i < slicedObject.vertices.size(); ++i) {
        const Vertex& v = slicedObject.vertices[i];
        vertices.push_back(v.getX());
        vertices.push_back(v.getY());
        vertices.push_back(v.getZ());
        vertexIndexMap[i] = currentIndex++;
    }

    for (const auto& face : slicedObject.faces) {
        const std::vector<int>& faceIndices = face.getVertexIndices();
        // Triangulate faces
        for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
            unsigned int idx0 = vertexIndexMap[faceIndices[0]];
            unsigned int idx1 = vertexIndexMap[faceIndices[i]];
            unsigned int idx2 = vertexIndexMap[faceIndices[i + 1]];
            indices.push_back(idx0);
            indices.push_back(idx1);
            indices.push_back(idx2);
        }
    }

    // Generate and bind VAO, VBO, EBO
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    updateBuffers();
}

void Renderer::updateBuffers()
{
    glBindVertexArray(vao);

    // Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Vertex attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Unbind VAO
    glBindVertexArray(0);
}
