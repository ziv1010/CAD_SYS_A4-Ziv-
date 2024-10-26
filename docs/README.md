# README

## 3D Model Reconstruction and Projection System

This system allows for the reconstruction of 3D models from 2D orthographic projections (top, front, and side views) and generates 2D projections from 3D models. It includes rendering capabilities for both 2D and 3D models, with interactive 3D visualization.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Input Formats](#input-formats)
  - [2D Input Format](#2d-input-format)
  - [3D Input Format](#3d-input-format)
- [Usage](#usage)
  - [Compiling the Code](#compiling-the-code)
  - [Running the Program](#running-the-program)
- [Examples](#examples)
- [Limitations](#limitations)
- [Support](#support)

---

## Prerequisites

Before running the system, ensure that you have the following installed on your machine:

- **C++ Compiler** with C++11 support (e.g., `g++`, `clang++`)
- **OpenGL Libraries** for rendering:
  - **GLEW** (OpenGL Extension Wrangler Library)
  - **GLFW** (Graphics Library Framework)
- **GLM** (OpenGL Mathematics Library) for mathematical computations
- **Python 3** (optional, for running the visualization scripts)
  - **Matplotlib** (if you wish to use the Python plotting scripts)

---

## Installation

### Installing Required Packages on Linux (Ubuntu/Debian)

1. **Update Package Lists:**

   ```bash
   sudo apt update
   ```

2. **Install Build-Essential Tools:**

   ```bash
   sudo apt install build-essential
   ```

3. **Install OpenGL Libraries:**

   ```bash
   sudo apt install libgl1-mesa-dev
   ```

4. **Install GLEW and GLFW:**

   ```bash
   sudo apt install libglew-dev libglfw3-dev
   ```

5. **Install GLM Library:**

   ```bash
   sudo apt install libglm-dev
   ```

6. **Install Python 3 and Matplotlib (Optional):**

   ```bash
   sudo apt install python3 python3-pip
   pip3 install matplotlib
   ```

### Installing Required Packages on macOS

1. **Install Homebrew** (if not already installed):

   ```bash
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```

2. **Install Packages via Homebrew:**

   ```bash
   brew install glew glfw glm
   ```

3. **Install Python 3 and Matplotlib (Optional):**

   ```bash
   brew install python3
   pip3 install matplotlib
   ```

### Installing Required Packages on Windows

1. **Install a C++ Compiler:**

   - Install [MinGW-w64](http://mingw-w64.org/doku.php) or use the compiler provided with Visual Studio.

2. **Install OpenGL Libraries:**

   - Download and install [GLEW](http://glew.sourceforge.net/) and [GLFW](https://www.glfw.org/).

3. **Install GLM Library:**

   - Download GLM from [here](https://github.com/g-truc/glm/releases) and include it in your project.

4. **Install Python 3 and Matplotlib (Optional):**

   - Download Python 3 from [python.org](https://www.python.org/downloads/windows/).
   - Install Matplotlib:

     ```bash
     pip install matplotlib
     ```

---

## Input Formats

### 2D Input Format

The 2D input file should start with the number `2`, indicating that it contains 2D data.

**Structure:**

```
2
[Direction]
[Number of Vertices]
[Vertex Data]
[Number of Edges]
[Edge Data]
[Repeat for each view (Top, Front, Side)]
```

**Details:**

- **Direction:** Integer indicating the view direction.
  - `0` - Top View
  - `1` - Front View
  - `2` - Side View
- **Vertex Data:** Each vertex is defined as:

  ```
  [vNo] [coord1] [coord2]
  ```

  - `vNo`: Vertex number (integer)
  - `coord1`, `coord2`: Coordinates in the 2D plane (floats)

- **Edge Data:** Each edge is defined as:

  ```
  [eno] [vNoA] [vNoB] [hidden]
  ```

  - `eno`: Edge number (integer)
  - `vNoA`, `vNoB`: Vertex numbers defining the edge
  - `hidden`: Visibility flag (`0` for visible, `1` for hidden)

**Example:**

```
2
0
4
1 0.0 0.0
2 1.0 0.0
3 1.0 1.0
4 0.0 1.0
4
1 1 2 0
2 2 3 0
3 3 4 0
4 4 1 0
1
4
1 0.0 0.0
2 1.0 0.0
3 1.0 1.0
4 0.0 1.0
4
1 1 2 0
2 2 3 0
3 3 4 0
4 4 1 0
2
...
```

### 3D Input Format

The 3D input file should start with the number `3`, indicating that it contains 3D data.

**Structure:**

```
3
[Number of Vertices]
[Vertex Data]
[Number of Edges]
[Edge Data]
[Number of Surfaces]
[Surface Data]
```

**Details:**

- **Vertex Data:** Each vertex is defined as:

  ```
  [vNo] [x] [y] [z]
  ```

  - `vNo`: Vertex number (integer)
  - `x`, `y`, `z`: Coordinates in 3D space (floats)

- **Edge Data:** Each edge is defined as:

  ```
  [eno] [vNoA] [vNoB]
  ```

  - `eno`: Edge number (integer)
  - `vNoA`, `vNoB`: Vertex numbers defining the edge

- **Surface Data:** Each surface is defined as:

  ```
  [sno] [Number of Boundary Edges] [eno1] [eno2] ... [enoN]
  ```

  - `sno`: Surface number (integer)
  - `Number of Boundary Edges`: Number of edges forming the surface boundary
  - `eno1`, `eno2`, ..., `enoN`: Edge numbers forming the boundary

**Example:**

```
3
4
1 0.0 0.0 0.0
2 1.0 0.0 0.0
3 1.0 1.0 0.0
4 0.0 1.0 0.0
4
1 1 2
2 2 3
3 3 4
4 4 1
1
1 4 1 2 3 4
```

---

## Usage

### Compiling the Code

1. **Navigate to the Project Directory:**

   ```bash
   cd /path/to/project
   ```

2. **Compile the Code:**

   ```bash
   g++ -std=c++11 main.cpp Renderer.cpp Vertex.cpp Edge.cpp Face.cpp 2Dto3DModel.cpp 3Dto2DModel.cpp -o model_converter -lGL -lGLEW -lglfw -lglm
   ```

   - Ensure all `.cpp` and `.h` files are in the same directory.
   - Adjust library links (`-lGL`, `-lGLEW`, `-lglfw`, `-lglm`) based on your system.

### Running the Program

The program can process both 2D and 3D input files.

1. **Run the Program with an Input File:**

   ```bash
   ./model_converter input.txt
   ```

   - Replace `input.txt` with the path to your input file.

2. **Follow On-Screen Prompts:**

   - The program may prompt for an output file path to save the results.
   - It will also display rendered models in a window.

3. **Interactive Controls for 3D Rendering:**

   - **Rotate Model:** Click and drag with the left mouse button.
   - **Zoom In/Out:** Use the mouse scroll wheel.
   - **Reset View:** Press the `R` key.

---

## Examples

### Example 1: Converting 2D Projections to 3D Model

1. **Prepare Input File (`2d_input.txt`):**

   - Include the 2D projections as per the 2D input format.

2. **Run the Program:**

   ```bash
   ./model_converter 2d_input.txt
   ```

3. **Output:**

   - The program generates a 3D model and saves it to the specified output file.
   - A window displays the reconstructed 3D model.

### Example 2: Generating 2D Projections from a 3D Model

1. **Prepare Input File (`3d_input.txt`):**

   - Include the 3D model data as per the 3D input format.

2. **Run the Program:**

   ```bash
   ./model_converter 3d_input.txt
   ```

3. **Output:**

   - The program generates 2D projections and saves them to the specified output file.
   - A window displays the generated 2D views.

---

## Limitations

- **Performance Constraints:**
  - The system may experience performance issues with extremely large models due to increased computational complexity.
  - Large input files can consume significant memory resources.

- **Unsupported Functionality:**
  - Transformation operations like rotation, translation, and scaling are not supported within the system (other than for rendering purposes).
  - Object segmentation (cutting the model into parts) is not implemented.
  - Extremely complex surfaces or non-manifold geometries may not be correctly processed.

- **Model Reusability:**
  - The output files generated by the system can be directly used as input files. This allows for iterative processing and testing without format conversion.
  - Users can input 2D projections, obtain a 3D model, and then use that 3D model as input to generate 2D projections again.

---



**Note:** Ensure that you have the necessary permissions and rights to use and modify the code and that you comply with any applicable licenses.

---