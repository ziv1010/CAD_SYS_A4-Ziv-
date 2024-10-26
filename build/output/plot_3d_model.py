import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # This import registers the 3D projection, but is otherwise unused.
import re
import os
import argparse

def parse_model_file(filepath):
    """
    Parses the 3D model file and extracts vertices and edges.

    Parameters:
        filepath (str): Path to the model file.

    Returns:
        vertices (dict): Mapping from vertex number to (x, y, z) coordinates.
        edges (list): List of tuples representing edges as (vNoA, vNoB).
    """
    vertices = {}
    edges = []

    if not os.path.isfile(filepath):
        print(f"Error: File '{filepath}' does not exist.")
        return vertices, edges

    with open(filepath, 'r') as file:
        lines = file.readlines()

    idx = 0  # Line index

    # Skip the first line (e.g., projection parameter)
    if idx >= len(lines):
        print("Error: File is empty.")
        return vertices, edges

    initial_param = lines[idx].strip()
    # You can use initial_param if needed
    # For now, we skip it
    idx += 1

    # Parse number of vertices
    if idx >= len(lines):
        print("Error: Unexpected end of file while reading number of vertices.")
        return vertices, edges

    try:
        N_v = int(lines[idx].strip())
    except ValueError:
        print(f"Error: Invalid number of vertices at line {idx+1}. Found: '{lines[idx].strip()}'")
        return vertices, edges

    idx += 1

    # Parse vertices
    for _ in range(N_v):
        if idx >= len(lines):
            print("Error: Unexpected end of file while reading vertices.")
            return vertices, edges

        line = lines[idx].strip()
        idx += 1

        # Expected format: vNo x y z
        match = re.match(r'^(\d+)\s+(-?\d*\.?\d+)\s+(-?\d*\.?\d+)\s+(-?\d*\.?\d+)$', line)
        if match:
            vNo = int(match.group(1))
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))
            vertices[vNo] = (x, y, z)
        else:
            print(f"Warning: Unable to parse vertex at line {idx}: '{line}'")

    # Parse number of edges
    if idx >= len(lines):
        print("Error: Unexpected end of file while reading number of edges.")
        return vertices, edges

    try:
        N_e = int(lines[idx].strip())
    except ValueError:
        print(f"Error: Invalid number of edges at line {idx+1}. Found: '{lines[idx].strip()}'")
        return vertices, edges

    idx += 1

    # Parse edges
    for _ in range(N_e):
        if idx >= len(lines):
            print("Error: Unexpected end of file while reading edges.")
            return vertices, edges

        line = lines[idx].strip()
        idx += 1

        # Expected format: eno vNoA vNoB
        match = re.match(r'^(\d+)\s+(\d+)\s+(\d+)$', line)
        if match:
            eno = int(match.group(1))  # Edge number, not used in plotting
            vNoA = int(match.group(2))
            vNoB = int(match.group(3))
            # Validate vertex numbers
            if vNoA in vertices and vNoB in vertices:
                edges.append( (vNoA, vNoB) )
            else:
                print(f"Warning: Edge {eno} references undefined vertices ({vNoA}, {vNoB}).")
        else:
            print(f"Warning: Unable to parse edge at line {idx}: '{line}'")

    # The rest of the file contains surfaces, which we ignore as per your request

    return vertices, edges

def plot_3d_model(vertices, edges, title="3D Model"):
    """
    Plots the 3D model using Matplotlib.

    Parameters:
        vertices (dict): Mapping from vertex number to (x, y, z) coordinates.
        edges (list): List of tuples representing edges as (vNoA, vNoB).
        title (str): Title of the plot.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot each edge
    for edge in edges:
        vNoA, vNoB = edge
        if vNoA in vertices and vNoB in vertices:
            x_vals = [vertices[vNoA][0], vertices[vNoB][0]]
            y_vals = [vertices[vNoA][1], vertices[vNoB][1]]
            z_vals = [vertices[vNoA][2], vertices[vNoB][2]]
            ax.plot(x_vals, y_vals, z_vals, color='b')  # Blue lines
        else:
            print(f"Warning: Edge ({vNoA}, {vNoB}) references undefined vertices.")

    # Set labels
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')

    # Set title
    ax.set_title(title)

    # Equal aspect ratio for all axes
    max_range = max(
        max([coord[0] for coord in vertices.values()]) - min([coord[0] for coord in vertices.values()]),
        max([coord[1] for coord in vertices.values()]) - min([coord[1] for coord in vertices.values()]),
        max([coord[2] for coord in vertices.values()]) - min([coord[2] for coord in vertices.values()])
    ) / 2

    mid_x = (max([coord[0] for coord in vertices.values()]) + min([coord[0] for coord in vertices.values()])) * 0.5
    mid_y = (max([coord[1] for coord in vertices.values()]) + min([coord[1] for coord in vertices.values()])) * 0.5
    mid_z = (max([coord[2] for coord in vertices.values()]) + min([coord[2] for coord in vertices.values()])) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()

def main():
    """
    Main function to execute the plotting process.
    """
    parser = argparse.ArgumentParser(description="Plot a 3D model from a custom-formatted file.")
    parser.add_argument('filepath', type=str, help='Path to the 3D model file.')
    args = parser.parse_args()

    filepath = args.filepath

    # Parse the model file
    vertices, edges = parse_model_file(filepath)

    if not vertices:
        print("No vertices to plot. Exiting.")
        return
    if not edges:
        print("No edges to plot. Exiting.")
        return

    # Plot the 3D model
    plot_3d_model(vertices, edges, title="3D Model without Hidden Lines")

if __name__ == "__main__":
    main()
