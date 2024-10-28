// renderer.cpp

#include "renderer.h"
#include <iostream>
#include <vector>
#include <map>
#include <cctype>

// Include GLM headers for transformations
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

const char* vertexShaderSource = R"(
// Vertex Shader
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor; // New: Color attribute

out vec3 vertexColor; // New: Pass color to fragment shader

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    vertexColor = aColor; // Pass color to fragment shader
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

const char* fragmentShaderSource = R"(
// Fragment Shader
#version 330 core
in vec3 vertexColor; // Receive color from vertex shader
out vec4 FragColor;

void main()
{
    FragColor = vec4(vertexColor, 1.0);
}
)";

Renderer::Renderer()
    : window(nullptr), shaderProgram(0),
      cameraZoom(60.0f), rotationAngle(0.0f),
      slicingAxis('X'), slicingPosition(0.0f)
{
}

Renderer::~Renderer()
{
    // Cleanup
    for (auto& obj : renderObjects) {
        glDeleteVertexArrays(1, &obj.vao);
        glDeleteBuffers(1, &obj.vbo);
        glDeleteBuffers(1, &obj.ebo);
    }
    glDeleteProgram(shaderProgram);
    glfwTerminate();
}

bool Renderer::initialize()
{
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // Request OpenGL 3.3 core profile context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // OpenGL major version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3); // OpenGL minor version
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Core profile

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
    glewExperimental = GL_TRUE; // Needed for core profiles
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl;
        return false;
    }

    // Configure OpenGL settings
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glDisable(GL_CULL_FACE); // Disable face culling to see all faces

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
        if (renderer->cameraZoom > 120.0f)
            renderer->cameraZoom = 120.0f;
    });

    // Set key callback for input handling
    glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        Renderer* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            renderer->handleSlicingInput();
        }
    });

    // Enable wireframe mode to visualize inner structures
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // Enable blending for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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

void Renderer::run()
{
    setupShaders();

    // Set up camera
    viewMatrix = glm::lookAt(glm::vec3(5.0f, 5.0f, 15.0f), // Adjusted camera position
                             glm::vec3(0.0f, 0.0f, 0.0f),
                             glm::vec3(0.0f, 1.0f, 0.0f));
    projectionMatrix = glm::perspective(glm::radians(cameraZoom), 800.0f / 600.0f, 0.1f, 100.0f);

    // Initialize rotation angle
    rotationAngle = 0.0f;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        processInput();

        // Clear the screen and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update rotation
        rotationAngle += 0.5f; // Adjust rotation speed as needed
        if (rotationAngle > 360.0f)
            rotationAngle -= 360.0f;

        // Update model matrix with rotation
        modelMatrix = glm::rotate(glm::mat4(1.0f), glm::radians(rotationAngle), glm::vec3(0.0f, 1.0f, 0.0f));

        // Update projection matrix in case cameraZoom changed
        projectionMatrix = glm::perspective(glm::radians(cameraZoom), 800.0f / 600.0f, 0.1f, 100.0f);

        // Use shader
        glUseProgram(shaderProgram);

        // Set uniforms
        GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
        GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
        GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
        // GLint colorLoc = glGetUniformLocation(shaderProgram, "objectColor"); // No longer needed

        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));

        // Draw each object
        for (const auto& renderObj : renderObjects) {
            glBindVertexArray(renderObj.vao);

            // Set object-specific color
            // glUniform3fv(colorLoc, 1, glm::value_ptr(renderObj.color)); // Not needed with per-vertex colors

            glDrawElements(GL_TRIANGLES, renderObj.indices.size(), GL_UNSIGNED_INT, 0);
        }

        // Draw the axes after drawing the objects
        drawAxes();

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
    bool slicingPlaneChanged = false;

    // Check for axis change
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        slicingAxis = 'X';
        slicingPlaneChanged = true;
    }
    if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS) {
        slicingAxis = 'Y';
        slicingPlaneChanged = true;
    }
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        slicingAxis = 'Z';
        slicingPlaneChanged = true;
    }

    // Adjust slicing position
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        slicingPosition += 0.01f; // Adjust increment as needed
        slicingPlaneChanged = true;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        slicingPosition -= 0.01f; // Adjust increment as needed
        slicingPlaneChanged = true;
    }

    if (slicingPlaneChanged) {
        // Update slicing plane
        setSlicingPlane(slicingAxis, slicingPosition);
        // Re-slice and update buffers
        sliceObject();
    }
}

void Renderer::sliceObject()
{
    // Slice the original object
    Object3D objectAbove, objectBelow;
    Slicer::sliceObject(originalObject, slicingPlane, objectAbove, objectBelow);

    // Clear previous render objects
    for (auto& obj : renderObjects) {
        glDeleteVertexArrays(1, &obj.vao);
        glDeleteBuffers(1, &obj.vbo);
        glDeleteBuffers(1, &obj.ebo);
    }
    renderObjects.clear();

    // Add the sliced outer cube
    addObject(objectBelow, glm::vec3(0.7f, 0.7f, 0.7f)); // Gray color

    // Optionally, attempt to distinguish inner cube
    // Since we cannot modify the input file, we can attempt to render the inner cube differently
    // For demonstration, we can render the entire object with a different color or use transparency
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

    // Specify the locations of the attributes
    glBindAttribLocation(shaderProgram, 0, "aPos");
    glBindAttribLocation(shaderProgram, 1, "aColor");

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

void Renderer::addObject(const Object3D& object, const glm::vec3& color) {
    RenderObject renderObj;
    renderObj.object = object;
    renderObj.color = color;

    // Convert object data to OpenGL buffers
    renderObj.vertices.clear();
    renderObj.indices.clear();

    // Map from Object3D vertex indices to OpenGL indices
    std::map<int, unsigned int> vertexIndexMap;

    unsigned int currentIndex = 0;
    for (size_t i = 0; i < object.vertices.size(); ++i) {
        const Vertex& v = object.vertices[i];
        renderObj.vertices.push_back(v.getX());
        renderObj.vertices.push_back(v.getY());
        renderObj.vertices.push_back(v.getZ());
        // Add color for each vertex (same color for the entire object)
        renderObj.vertices.push_back(color.r);
        renderObj.vertices.push_back(color.g);
        renderObj.vertices.push_back(color.b);
        vertexIndexMap[i] = currentIndex++;
    }

    for (const auto& face : object.faces) {
        const std::vector<int>& faceIndices = face.getVertexIndices();
        // Triangulate faces
        for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
            unsigned int idx0 = vertexIndexMap[faceIndices[0]];
            unsigned int idx1 = vertexIndexMap[faceIndices[i]];
            unsigned int idx2 = vertexIndexMap[faceIndices[i + 1]];
            renderObj.indices.push_back(idx0);
            renderObj.indices.push_back(idx1);
            renderObj.indices.push_back(idx2);
        }
    }

    // Generate and bind VAO, VBO, EBO
    glGenVertexArrays(1, &renderObj.vao);
    glGenBuffers(1, &renderObj.vbo);
    glGenBuffers(1, &renderObj.ebo);

    glBindVertexArray(renderObj.vao);

    // Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, renderObj.vbo);
    glBufferData(GL_ARRAY_BUFFER, renderObj.vertices.size() * sizeof(float), renderObj.vertices.data(), GL_STATIC_DRAW);

    // Element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderObj.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, renderObj.indices.size() * sizeof(unsigned int), renderObj.indices.data(), GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Unbind VAO
    glBindVertexArray(0);

    // Add to renderObjects list
    renderObjects.push_back(renderObj);
}

// **New function to draw axes with unit markers**
void Renderer::drawAxes() {
    // Define vertices for the axes
    float axisLength = 5.0f; // Adjust as needed
    float axisVertices[] = {
        // Positions             // Colors (R, G, B)
        // X-axis (Red)
        0.0f, 0.0f, 0.0f,        1.0f, 0.0f, 0.0f, // Origin
        axisLength, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f, // X-axis endpoint

        // Y-axis (Green)
        0.0f, 0.0f, 0.0f,        0.0f, 1.0f, 0.0f, // Origin
        0.0f, axisLength, 0.0f,  0.0f, 1.0f, 0.0f, // Y-axis endpoint

        // Z-axis (Blue)
        0.0f, 0.0f, 0.0f,        0.0f, 0.0f, 1.0f, // Origin
        0.0f, 0.0f, axisLength,  0.0f, 0.0f, 1.0f  // Z-axis endpoint
    };

    GLuint axisVAO, axisVBO;
    glGenVertexArrays(1, &axisVAO);
    glGenBuffers(1, &axisVBO);

    glBindVertexArray(axisVAO);

    glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Use shader program
    glUseProgram(shaderProgram);

    // Set uniforms
    GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");

    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewMatrix));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelMatrix));

    // Draw the axes
    glBindVertexArray(axisVAO);
    glDrawArrays(GL_LINES, 0, 6); // 6 vertices, 2 per axis

    // Create unit markers for each axis
    std::vector<float> unitMarkers;

    // X-axis unit markers (Red)
    for (int i = 1; i <= (int)axisLength; ++i) {
        unitMarkers.push_back((float)i); // x
        unitMarkers.push_back(0.0f);     // y
        unitMarkers.push_back(0.0f);     // z
        unitMarkers.push_back(1.0f);     // r
        unitMarkers.push_back(0.0f);     // g
        unitMarkers.push_back(0.0f);     // b
    }

    // Y-axis unit markers (Green)
    for (int i = 1; i <= (int)axisLength; ++i) {
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back((float)i);
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back(1.0f);
        unitMarkers.push_back(0.0f);
    }

    // Z-axis unit markers (Blue)
    for (int i = 1; i <= (int)axisLength; ++i) {
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back((float)i);
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back(0.0f);
        unitMarkers.push_back(1.0f);
    }

    // Set up VBO and VAO for unit markers
    GLuint markersVAO, markersVBO;
    glGenVertexArrays(1, &markersVAO);
    glGenBuffers(1, &markersVBO);

    glBindVertexArray(markersVAO);

    glBindBuffer(GL_ARRAY_BUFFER, markersVBO);
    glBufferData(GL_ARRAY_BUFFER, unitMarkers.size() * sizeof(float), unitMarkers.data(), GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Draw unit markers as points
    glPointSize(5.0f); // Adjust point size as needed
    glBindVertexArray(markersVAO);
    glDrawArrays(GL_POINTS, 0, unitMarkers.size() / 6);

    // Cleanup
    glBindVertexArray(0);
    glDeleteVertexArrays(1, &markersVAO);
    glDeleteBuffers(1, &markersVBO);

    glDeleteVertexArrays(1, &axisVAO);
    glDeleteBuffers(1, &axisVBO);
}
