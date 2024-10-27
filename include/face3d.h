// face3d.h
#ifndef FACE3D_H
#define FACE3D_H

#include <vector>
#include <string>
#include <cstdio>

enum class FaceStatus {
    UNDECIDED,
    TRUE_FACE,
    FALSE_FACE
};

class Face3D {
public:
    std::vector<int> vertexIndices;
    std::vector<int> edgeIndices; // Indices of edges forming the face
    FaceStatus status;

    Face3D() : status(FaceStatus::UNDECIDED) {}

    Face3D(const std::vector<int>& indices) : vertexIndices(indices), status(FaceStatus::UNDECIDED) {}

    void print() const {
        printf("Face3D with vertices:");
        for (int idx : vertexIndices) {
            printf(" %d", idx);
        }
        printf(" [Status: %s]\n", statusToString().c_str());
    }

    // In face3d.h
    std::string statusToString() const {
        switch (status) {
            case FaceStatus::UNDECIDED: return "UNDECIDED";
            case FaceStatus::TRUE_FACE: return "TRUE";
            case FaceStatus::FALSE_FACE: return "FALSE";
            default: return "UNKNOWN";
        }
    }

};

#endif // FACE3D_H
