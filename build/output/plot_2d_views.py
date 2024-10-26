import matplotlib.pyplot as plt
import argparse
import os

def parse_2d_model_file(filepath):
    """
    Parses the 2D model file and extracts the views.

    Parameters:
        filepath (str): Path to the model file.

    Returns:
        views (dict): Mapping from view_type to {'vertices': {vNo: (coord1, coord2)}, 'edges': [(vNoA, vNoB)]}
    """
    views = {}

    if not os.path.isfile(filepath):
        print(f"Error: File '{filepath}' does not exist.")
        return views

    with open(filepath, 'r') as f:
        lines = f.readlines()

    idx = 0
    total_lines = len(lines)

    # Read first line: '2' for 2D data
    if idx >= total_lines:
        print("Error: Empty file.")
        return views
    dim = lines[idx].strip()
    if dim != '2':
        print("Error: File does not represent 2D data. First line should be '2'.")
        return views
    idx += 1

    while idx < total_lines:
        # Read view_type
        view_type_line = lines[idx].strip()
        if not view_type_line:
            print(f"Warning: Empty line at line {idx+1}. Skipping.")
            idx += 1
            continue
        try:
            view_type = int(view_type_line)
        except ValueError:
            print(f"Warning: Invalid view type '{view_type_line}' at line {idx+1}. Skipping.")
            idx += 1
            continue
        if view_type not in [0, 1, 2]:
            print(f"Warning: Unknown view type '{view_type}' at line {idx+1}. Skipping.")
            idx += 1
            continue
        idx += 1

        # Read number of vertices
        if idx >= total_lines:
            print("Error: Unexpected end of file while reading number of vertices.")
            break
        num_vertices_line = lines[idx].strip()
        if not num_vertices_line:
            print(f"Warning: Empty line at line {idx+1} while reading number of vertices. Skipping.")
            idx += 1
            continue
        try:
            num_vertices = int(num_vertices_line)
        except ValueError:
            print(f"Error: Invalid number of vertices '{num_vertices_line}' at line {idx+1}. Skipping.")
            break
        idx += 1

        # Read vertices
        vertices = {}
        for _ in range(num_vertices):
            if idx >= total_lines:
                print("Error: Unexpected end of file while reading vertices.")
                break
            vertex_line = lines[idx].strip()
            idx += 1
            if not vertex_line:
                print(f"Warning: Empty line at line {idx}. Skipping vertex.")
                continue
            parts = vertex_line.split()
            if len(parts) < 3:
                print(f"Warning: Incomplete vertex data '{vertex_line}' at line {idx}. Skipping.")
                continue
            vNo = int(parts[0])
            coord1 = float(parts[1])
            coord2 = float(parts[2])
            vertices[vNo] = (coord1, coord2)

        # Read number of edges
        if idx >= total_lines:
            print("Error: Unexpected end of file while reading number of edges.")
            break
        num_edges_line = lines[idx].strip()
        if not num_edges_line:
            print(f"Warning: Empty line at line {idx+1} while reading number of edges. Skipping.")
            idx += 1
            continue
        try:
            num_edges = int(num_edges_line)
        except ValueError:
            print(f"Error: Invalid number of edges '{num_edges_line}' at line {idx+1}. Skipping.")
            break
        idx += 1

        # Read edges
        edges = []
        for _ in range(num_edges):
            if idx >= total_lines:
                print("Error: Unexpected end of file while reading edges.")
                break
            edge_line = lines[idx].strip()
            idx += 1
            if not edge_line:
                print(f"Warning: Empty line at line {idx}. Skipping edge.")
                continue
            parts = edge_line.split()
            if len(parts) < 4:
                print(f"Warning: Incomplete edge data '{edge_line}' at line {idx}. Skipping.")
                continue
            eNo = int(parts[0])
            vNoA = int(parts[1])
            vNoB = int(parts[2])
            # h = int(parts[3])  # Hidden flag, ignored
            edges.append( (vNoA, vNoB) )

        # Store in views
        views[view_type] = {'vertices': vertices, 'edges': edges}

    return views

def plot_views(views):
    """
    Plots the Top, Front, and Side views of the model.

    Parameters:
        views (dict): Mapping from view_type to {'vertices': {vNo: (coord1, coord2)}, 'edges': [(vNoA, vNoB)]}
    """
    # Create a figure with 1 row and 3 columns for the three views
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))

    # Define the mapping from view_type to subplot index and labels
    view_labels = {
        0: 'Top View (X-Y)',
        1: 'Front View (X-Z)',
        2: 'Side View (Y-Z)'
    }
    subplot_mapping = {
        0: axs[0],
        1: axs[1],
        2: axs[2]
    }

    for view_type, data in views.items():
        ax = subplot_mapping.get(view_type)
        if not ax:
            print(f"Warning: No subplot assigned for view type {view_type}. Skipping.")
            continue
        vertices = data['vertices']
        edges = data['edges']

        # Determine axis labels based on view_type
        if view_type == 0:
            xlabel = 'X'
            ylabel = 'Y'
        elif view_type == 1:
            xlabel = 'X'
            ylabel = 'Z'
        elif view_type == 2:
            xlabel = 'Y'
            ylabel = 'Z'
        else:
            xlabel = 'X'
            ylabel = 'Y'

        # Plot all edges
        for edge in edges:
            vNoA, vNoB = edge
            if vNoA in vertices and vNoB in vertices:
                coordA = vertices[vNoA]
                coordB = vertices[vNoB]
                if view_type == 0:
                    x_vals = [coordA[0], coordB[0]]
                    y_vals = [coordA[1], coordB[1]]
                elif view_type == 1:
                    x_vals = [coordA[0], coordB[0]]
                    y_vals = [coordA[1], coordB[1]]  # Y represents Z
                elif view_type == 2:
                    x_vals = [coordA[0], coordB[0]]  # X represents Y
                    y_vals = [coordA[1], coordB[1]]  # Y represents Z
                else:
                    x_vals = [coordA[0], coordB[0]]
                    y_vals = [coordA[1], coordB[1]]
                ax.plot(x_vals, y_vals, color='blue')
            else:
                print(f"Warning: Edge ({vNoA}, {vNoB}) references undefined vertices.")

        # Set titles and labels
        ax.set_title(view_labels.get(view_type, f'View {view_type}'))
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_aspect('equal', adjustable='box')
        ax.grid(True)

    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()

def main():
    """
    Main function to execute the plotting process.
    """
    parser = argparse.ArgumentParser(description="Plot 2D views of a model.")
    parser.add_argument('filepath', type=str, help='Path to the 2D model file.')
    args = parser.parse_args()

    filepath = args.filepath
    views = parse_2d_model_file(filepath)

    if not views:
        print("No views to plot. Exiting.")
        return

    plot_views(views)

if __name__ == "__main__":
    main()
