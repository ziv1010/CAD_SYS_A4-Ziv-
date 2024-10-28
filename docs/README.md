# 3D to 2D Projection and 2D to 3D Reconstruction System

This project contains two programs:

1. **3D to 2D Projection Program**: Performs geometric transformations, slicing, and generates orthographic projections (top, front, side views) from a 3D object. It also calculates the surface area and volume of the 3D object.

2. **2D to 3D Reconstruction Program**: Reconstructs a 3D wireframe model from three orthographic 2D projections (Top View, Front View, and Side View).

## Features

### 3D to 2D Projection Program

- Reads a 3D object from an input file.
- Applies geometric transformations: rotation, translation, scaling.
- Performs slicing of the object with a plane.
- Generates orthographic projections with hidden line removal.
- Calculates surface area and volume.
- Visualizes the 3D object using OpenGL.

### 2D to 3D Reconstruction Program

- Reads 2D projections from input files.
- Processes the input data to remove duplicates and handle intersections.
- Constructs probable 3D vertices and edges.
- Validates the vertices and edges to remove pseudo-elements.
- Generates a 3D wireframe model.

## Installation

### Dependencies

The project requires the following dependencies:

- **OpenGL**: Cross-language, cross-platform API for rendering 2D and 3D vector graphics.
- **GLFW**: Open Source, multi-platform library for OpenGL, OpenGL ES, and Vulkan development on the desktop.
- **GLEW**: The OpenGL Extension Wrangler Library.
- **GLM**: OpenGL Mathematics library.

#### Installing Dependencies on Linux (Ubuntu/Debian)

You can install the required dependencies using your package manager:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake
sudo apt-get install libglfw3-dev libglew-dev libglm-dev
sudo apt-get install freeglut3-dev
```

#### Installing Dependencies on Windows

- Install [Visual Studio](https://visualstudio.microsoft.com/) for C++ development.
- Download and install the precompiled binaries for GLFW and GLEW.
- Ensure that the include directories and libraries are correctly set in your project settings.

#### Installing Dependencies on macOS

```bash
brew install cmake glfw glew glm
```

### Building the Project

1. **Clone the Repository**

   ```bash
   git clone https://github.com/yourusername/yourrepository.git
   cd yourrepository
   ```

2. **Create Build Directory**

   ```bash
   mkdir build
   cd build
   ```

3. **Configure the Project with CMake**

   ```bash
   cmake ..
   ```

4. **Build the Project**

   ```bash
   make
   ```

   This will compile the code and create executables in the `build/output/` directory.

## Usage

### Running the 3D to 2D Projection Program

1. **Prepare the Input File**

   - Place your 3D object file (e.g., `input3Dcube.txt`) in the `build/output/` directory.
   - Ensure the file follows the specified 3D Object Input Format (see [Input File Formats](#input-file-formats)).

2. **Specify the Input File in the Code**

   - Open `main.cpp` in the `src/` directory with your preferred text editor.
   - Locate the line where the input file is specified:

     ```cpp
     read3DObjectFromFile("build/output/input3Dcube.txt", originalObject);
     ```

   - Change `"build/output/input3Dcube.txt"` to the path of your input file if necessary.

3. **Compile and Run the Program**

   From the `build/` directory:

   ```bash
   make
   ./output/app
   ```

   - The program will display a menu in the terminal. Follow the prompts to perform transformations, slicing, or projections.

4. **View the Output**

   - The program will generate output files in the `build/output/` directory, including images (`.png`) and text files (`.txt`) of the projections.

### Running the 2D to 3D Reconstruction Program

1. **Prepare the Input File**

   - Place your 2D projections file (e.g., `input2d.txt`) in the `build/output/` directory.
   - Ensure the file follows the specified 2D Projection Input Format (see [Input File Formats](#input-file-formats)).

2. **Specify the Input File in the Code**

   - Open `main.cpp` in the `src/` directory with your preferred text editor.
   - Locate the line where the input file is specified:

     ```cpp
     readGraphsFromFile("build/output/input2d.txt", topView, frontView, sideView);
     ```

   - Change `"build/output/input2d.txt"` to the path of your input file if necessary.

3. **Compile and Run the Program**

   From the `build/` directory:

   ```bash
   make
   ./output/app
   ```

   - The program will process the input data and reconstruct the 3D wireframe model.

4. **View the Output**

   - The program will generate an output file (e.g., `output.txt`) in the `build/output/` directory containing the reconstructed 3D model data.

## Input File Formats

### 3D Object Input Format

The 3D object file should follow this format:

1. **Number of Vertices (V)**: An integer specifying the total number of vertices.

2. **Vertex Coordinates**: V lines, each containing three floating-point numbers representing x, y, z coordinates.

3. **Number of Edges (E)**: An integer specifying the total number of edges.

4. **Edge Indices**: E lines, each containing two integers representing indices of the vertices that form the edge.

5. **Number of Faces (F)**: An integer specifying the total number of faces.

6. **Faces Definition**: F blocks, each starting with an integer N (number of vertices in the face), followed by N integers representing vertex indices.

**Example:**

```
8
0 0 0
1 0 0
1 1 0
0 1 0
0 0 1
1 0 1
1 1 1
0 1 1
12
0 1
1 2
2 3
3 0
4 5
5 6
6 7
7 4
0 4
1 5
2 6
3 7
6
4 0 1 2 3
4 4 5 6 7
4 0 1 5 4
4 1 2 6 5
4 2 3 7 6
4 3 0 4 7
```

### 2D Projection Input Format

The 2D projections file should follow this format for each view:

1. **View Name**: Either `Top_View`, `Front_View`, or `Side_View`.

2. **Number of Vertices (V)**: An integer specifying the total number of vertices.

3. **Vertex Coordinates**: V lines, each containing two floating-point numbers representing x and y coordinates.

4. **Number of Edges (E)**: An integer specifying the total number of edges.

5. **Edge Definitions**: E lines, each containing two integers representing the indices of the start and end vertices and a string indicating the line type (`solid` or `dashed`).

**Example:**

```
Top_View
5
0 0
1 0
1 1
0 1
0.5 0.5
6
0 1 solid
1 2 solid
2 3 solid
3 0 solid
0 4 dashed
2 4 dashed
Front_View
...
Side_View
...
```

Repeat this format for each view in the same file.

## Code Structure

- **src/**: Contains all the source code files.
  - **main.cpp**: Entry point of the application.
  - **object3d.h/.cpp**: Definition and implementation of the `Object3D` class for 3D objects.
  - **vertex3d.h/.cpp**: Definition and implementation of the `Vertex3D` class.
  - **edge3d.h/.cpp**: Definition and implementation of the `Edge3D` class.
  - **face3d.h/.cpp**: Definition and implementation of the `Face3D` class.
  - **graph2d.h/.cpp**: Classes and methods for handling 2D projections.
  - **wireframe_model.h/.cpp**: Methods for reconstructing the 3D wireframe model from 2D projections.
  - **renderer.h/.cpp**: Handles visualization using OpenGL.
  - **transformations.h/.cpp**: Contains geometric transformation functions.
  - **file_io.h/.cpp**: Functions for reading and writing data to files.
- **build/**: Build directory containing compiled binaries and output files.
  - **output/**: Directory where output images and text files are saved.
- **CMakeLists.txt**: Build configuration file for CMake.

## Notes

- **Choosing Input Files**: Before running the programs, ensure you specify the correct input file paths in the `main.cpp` files.
- **Running the Executable**: After building, the executable can be run from the `build/` directory using:

  ```bash
  ./output/app
  ```

- **Rebuilding the Project**: If you make changes to the source code or input files, recompile using:

  ```bash
  make
  ```

## License

[Specify your project's license here, e.g., MIT License.]

## Acknowledgments

- [GLFW](https://www.glfw.org/)
- [GLEW](http://glew.sourceforge.net/)
- [OpenGL](https://www.opengl.org/)
- [GLM](https://github.com/g-truc/glm)

---
