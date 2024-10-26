#include "edge3d.h"

Edge3D::Edge3D(int start_idx, int end_idx)
    : start_vertex_index(start_idx), end_vertex_index(end_idx), examined(false) {}