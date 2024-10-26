#ifndef FILE_IO_H
#define FILE_IO_H

#include "object3d.h"
#include <string>

void read3DObjectFromFile(const std::string& filename, Object3D& object);

#endif // FILE_IO_H