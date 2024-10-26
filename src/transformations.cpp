#include "transformations.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

void Transformations::rotateX(Object3D& object, float angle) {
    float radians = glm::radians(angle);
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), radians, glm::vec3(1, 0, 0));

    for (auto& vertex : object.vertices) {
        glm::vec4 pos(vertex.getX(), vertex.getY(), vertex.getZ(), 1.0f);
        pos = rotation * pos;
        vertex.setX(pos.x);
        vertex.setY(pos.y);
        vertex.setZ(pos.z);
    }
}


void Transformations::rotateY(Object3D& object, float angle) {
    float radians = glm::radians(angle);
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), radians, glm::vec3(0, 1, 0));

    for (auto& vertex : object.vertices) {
        glm::vec4 pos(vertex.getX(), vertex.getY(), vertex.getZ(), 1.0f);
        pos = rotation * pos;
        vertex.setX(pos.x);
        vertex.setY(pos.y);
        vertex.setZ(pos.z);
    }
}

void Transformations::rotateZ(Object3D& object, float angle) {
    float radians = glm::radians(angle);
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), radians, glm::vec3(0, 0, 1));

    for (auto& vertex : object.vertices) {
        glm::vec4 pos(vertex.getX(), vertex.getY(), vertex.getZ(), 1.0f);
        pos = rotation * pos;
        vertex.setX(pos.x);
        vertex.setY(pos.y);
        vertex.setZ(pos.z);
    }
}

// Translate the object along X, Y, Z axes
void Transformations::translate(Object3D& object, float tx, float ty, float tz) {
    for (auto& vertex : object.vertices) {
        vertex.setX(vertex.getX() + tx);
        vertex.setY(vertex.getY() + ty);
        vertex.setZ(vertex.getZ() + tz);
    }
}

// Uniform Scaling
void Transformations::scale(Object3D& object, float scaleFactor) {
    for (auto& vertex : object.vertices) {
        vertex.setX(vertex.getX() * scaleFactor);
        vertex.setY(vertex.getY() * scaleFactor);
        vertex.setZ(vertex.getZ() * scaleFactor);
    }
}