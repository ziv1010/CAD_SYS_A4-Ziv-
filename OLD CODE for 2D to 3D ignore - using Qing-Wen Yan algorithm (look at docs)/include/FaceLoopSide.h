// FaceLoopSide.h
#ifndef FACELOOPSIDE_H
#define FACELOOPSIDE_H

struct FaceLoopSide {
    int planeIndex;
    int faceLoopIndex;
    bool positiveSide; // true for positive side, false for negative side

    FaceLoopSide(int pIndex = -1, int fIndex = -1, bool posSide = true)
        : planeIndex(pIndex), faceLoopIndex(fIndex), positiveSide(posSide) {}

    // Other members and methods if necessary
};

#endif // FACELOOPSIDE_H
