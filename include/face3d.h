#ifndef FACE3D_H
#define FACE3D_H

#include <vector>
#include <cstdio>

class Face3D {
public:
    std::vector<int> vertexIndices;

    Face3D() {}

    Face3D(const std::vector<int>& indices) : vertexIndices(indices) {}

    // Debug print
    void print() const {
        printf("Face3D with vertices:");
        for (int idx : vertexIndices) {
            printf(" %d", idx);
        }
        printf("\n");
    }
};

#endif // FACE3D_H
