#!/usr/bin/env python3
"""
Nerve-Tumor Visualization Script
Visualizes both the neuroma tumor mesh and the generated nerve rod for verification

Dependencies: matplotlib, numpy
Install with: pip install matplotlib numpy
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_tumor_file(tumor_file, max_vertices=None):
    """Parse tumor file (either .msh or .obj format)"""
    file_ext = os.path.splitext(tumor_file)[1].lower()
    
    if file_ext == '.obj':
        return parse_obj_file(tumor_file, max_vertices)
    elif file_ext == '.msh':
        return parse_gmsh_nodes(tumor_file, max_vertices)
    else:
        print(f"Unsupported file format: {file_ext}")
        return np.array([])

def parse_obj_file(obj_file, max_vertices=None):
    """Parse OBJ file to extract vertex coordinates"""
    vertices = []
    
    try:
        with open(obj_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('v '):  # Vertex line
                    parts = line.split()
                    if len(parts) >= 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        vertices.append([x, y, z])
                        if max_vertices and len(vertices) >= max_vertices:
                            break
        
        print(f"Parsing {obj_file} (OBJ format)")
        print(f"  Loaded: {len(vertices)} vertices")
        return np.array(vertices)
        
    except Exception as e:
        print(f"Error parsing OBJ file: {e}")
        return np.array([])

def parse_gmsh_nodes(msh_file, max_vertices=None):
    """Parse Gmsh .msh file to extract vertex coordinates"""
    vertices = []
    
    try:
        with open(msh_file, 'r') as f:
            lines = f.readlines()
        
        # Detect version
        version = "2.2"
        for line in lines:
            if line.strip().startswith('$MeshFormat'):
                continue
            elif line.strip() and not line.startswith('$'):
                version_info = line.strip().split()
                if len(version_info) > 0:
                    version = version_info[0]
                    break
        
        print(f"Parsing {msh_file} (Gmsh {version})")
        
        # Find $Nodes section
        for i, line in enumerate(lines):
            if line.strip() == '$Nodes':
                if version.startswith('4'):
                    # Gmsh 4.x format
                    header_parts = lines[i + 1].strip().split()
                    if len(header_parts) >= 2:
                        total_nodes = int(header_parts[1])
                        print(f"  Total nodes: {total_nodes}")
                        
                        line_idx = i + 2
                        nodes_read = 0
                        
                        while line_idx < len(lines) and nodes_read < total_nodes:
                            if max_vertices and nodes_read >= max_vertices:
                                break
                                
                            line_content = lines[line_idx].strip()
                            if line_content == '$EndNodes':
                                break
                            
                            # Parse entity block header
                            entity_parts = line_content.split()
                            if len(entity_parts) == 4 and entity_parts[0].isdigit():
                                entity_dim, entity_tag, parametric, num_nodes_in_block = map(int, entity_parts)
                                line_idx += 1
                                
                                # Skip node tags
                                for tag_offset in range(num_nodes_in_block):
                                    if line_idx + tag_offset < len(lines):
                                        tag_line = lines[line_idx + tag_offset].strip()
                                        if not tag_line.isdigit():
                                            break
                                line_idx += num_nodes_in_block
                                
                                # Read coordinates
                                for coord_offset in range(num_nodes_in_block):
                                    if max_vertices and nodes_read >= max_vertices:
                                        break
                                    coord_line_idx = line_idx + coord_offset
                                    if coord_line_idx < len(lines):
                                        coord_parts = lines[coord_line_idx].strip().split()
                                        if len(coord_parts) >= 3:
                                            try:
                                                x, y, z = float(coord_parts[0]), float(coord_parts[1]), float(coord_parts[2])
                                                vertices.append([x, y, z])
                                                nodes_read += 1
                                            except ValueError:
                                                continue
                                
                                line_idx += num_nodes_in_block
                            else:
                                line_idx += 1
                        break
                else:
                    # Gmsh 2.x format
                    num_nodes = int(lines[i + 1].strip())
                    print(f"  Nodes: {num_nodes}")
                    
                    for j in range(i + 2, min(i + 2 + num_nodes, len(lines))):
                        if max_vertices and len(vertices) >= max_vertices:
                            break
                        parts = lines[j].strip().split()
                        if len(parts) >= 4:  # node_id x y z
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            vertices.append([x, y, z])
                    break
        
        print(f"  Loaded: {len(vertices)} vertices")
        return np.array(vertices)
        
    except Exception as e:
        print(f"Error parsing {msh_file}: {e}")
        return np.array([])

def parse_nerve_rod_geo(geo_file):
    """Parse .geo file to extract nerve rod points"""
    points = []
    
    try:
        with open(geo_file, 'r') as f:
            lines = f.readlines()
        
        for line in lines:
            line = line.strip()
            if line.startswith('Point(') and '=' in line:
                # Parse: Point(1) = {x, y, z, size};
                coords_part = line.split('=')[1].strip()
                if coords_part.startswith('{') and coords_part.endswith('};'):
                    coords_str = coords_part[1:-2]  # Remove { and };
                    coords = [float(x.strip()) for x in coords_str.split(',')[:3]]  # Take first 3 (x,y,z)
                    points.append(coords)
        
        print(f"Parsed nerve rod: {len(points)} points")
        return np.array(points)
        
    except Exception as e:
        print(f"Error parsing {geo_file}: {e}")
        return np.array([])

def create_3d_visualization(tumor_vertices, nerve_points, title="Nerve-Tumor Assembly"):
    """Create 3D visualization of tumor and nerve"""
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot tumor vertices (sample for performance)
    if len(tumor_vertices) > 0:
        # Sample tumor vertices for visualization (too many to plot all)
        if len(tumor_vertices) > 5000:
            indices = np.random.choice(len(tumor_vertices), 5000, replace=False)
            tumor_sample = tumor_vertices[indices]
        else:
            tumor_sample = tumor_vertices
        
        ax.scatter(tumor_sample[:, 0], tumor_sample[:, 1], tumor_sample[:, 2], 
                  c='lightgreen', s=1, alpha=0.3, label=f'Tumor ({len(tumor_vertices)} vertices)')
    
    # Plot nerve rod
    if len(nerve_points) > 0:
        # Plot nerve points
        ax.scatter(nerve_points[:, 0], nerve_points[:, 1], nerve_points[:, 2], 
                  c='red', s=50, alpha=0.8, label=f'Nerve ({len(nerve_points)} points)')
        
        # Plot nerve rod as connected line
        ax.plot(nerve_points[:, 0], nerve_points[:, 1], nerve_points[:, 2], 
               'r-', linewidth=3, alpha=0.9, label='Nerve Rod')
    
    # Calculate and display bounding boxes
    if len(tumor_vertices) > 0:
        tumor_min = np.min(tumor_vertices, axis=0)
        tumor_max = np.max(tumor_vertices, axis=0)
        tumor_center = (tumor_min + tumor_max) / 2
        tumor_size = tumor_max - tumor_min
        
        print(f"Tumor bounding box:")
        print(f"  Min: [{tumor_min[0]:.1f}, {tumor_min[1]:.1f}, {tumor_min[2]:.1f}]")
        print(f"  Max: [{tumor_max[0]:.1f}, {tumor_max[1]:.1f}, {tumor_max[2]:.1f}]")
        print(f"  Size: [{tumor_size[0]:.1f}, {tumor_size[1]:.1f}, {tumor_size[2]:.1f}]")
        
        # Draw bounding box
        bbox_corners = np.array([
            [tumor_min[0], tumor_min[1], tumor_min[2]], [tumor_max[0], tumor_min[1], tumor_min[2]],
            [tumor_max[0], tumor_max[1], tumor_min[2]], [tumor_min[0], tumor_max[1], tumor_min[2]],
            [tumor_min[0], tumor_min[1], tumor_max[2]], [tumor_max[0], tumor_min[1], tumor_max[2]],
            [tumor_max[0], tumor_max[1], tumor_max[2]], [tumor_min[0], tumor_max[1], tumor_max[2]]
        ])
        
        # Draw bounding box edges
        bbox_edges = [
            [0,1], [1,2], [2,3], [3,0],  # Bottom face
            [4,5], [5,6], [6,7], [7,4],  # Top face
            [0,4], [1,5], [2,6], [3,7]   # Vertical edges
        ]
        
        for edge in bbox_edges:
            ax.plot3D(*bbox_corners[edge].T, 'k--', alpha=0.5, linewidth=1)
    
    if len(nerve_points) > 0:
        nerve_min = np.min(nerve_points, axis=0)
        nerve_max = np.max(nerve_points, axis=0)
        
        print(f"Nerve extent:")
        print(f"  Min: [{nerve_min[0]:.1f}, {nerve_min[1]:.1f}, {nerve_min[2]:.1f}]")
        print(f"  Max: [{nerve_max[0]:.1f}, {nerve_max[1]:.1f}, {nerve_max[2]:.1f}]")
    
    # Set labels and title
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title(title)
    ax.legend()
    
    # Set equal aspect ratio
    if len(tumor_vertices) > 0:
        max_range = np.max(tumor_size) / 2
        ax.set_xlim(tumor_center[0] - max_range, tumor_center[0] + max_range)
        ax.set_ylim(tumor_center[1] - max_range, tumor_center[1] + max_range)
        ax.set_zlim(tumor_center[2] - max_range, tumor_center[2] + max_range)
    
    plt.tight_layout()
    return fig, ax

def create_2d_projections(tumor_vertices, nerve_points, title="Nerve-Tumor Projections"):
    """Create 2D projection views (XY, XZ, YZ)"""
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    projections = [
        ('X-Y View', 0, 1),
        ('X-Z View', 0, 2),
        ('Y-Z View', 1, 2)
    ]
    
    for i, (view_name, xi, yi) in enumerate(projections):
        ax = axes[i]
        
        # Plot tumor projection
        if len(tumor_vertices) > 0:
            # Sample for performance
            if len(tumor_vertices) > 2000:
                indices = np.random.choice(len(tumor_vertices), 2000, replace=False)
                tumor_sample = tumor_vertices[indices]
            else:
                tumor_sample = tumor_vertices
            
            ax.scatter(tumor_sample[:, xi], tumor_sample[:, yi], 
                      c='lightgreen', s=1, alpha=0.4, label='Tumor')
        
        # Plot nerve projection
        if len(nerve_points) > 0:
            ax.scatter(nerve_points[:, xi], nerve_points[:, yi], 
                      c='red', s=30, alpha=0.8, label='Nerve Points')
            ax.plot(nerve_points[:, xi], nerve_points[:, yi], 
                   'r-', linewidth=2, alpha=0.9, label='Nerve Rod')
        
        ax.set_xlabel(['X (cm)', 'X (cm)', 'Y (cm)'][i])
        ax.set_ylabel(['Y (cm)', 'Z (cm)', 'Z (cm)'][i])
        ax.set_title(view_name)
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='box')
    
    plt.suptitle(title)
    plt.tight_layout()
    return fig, axes

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 visualize_nerve_tumor.py <tumor.msh|obj> <nerve.geo> [--save-images]")
        print("")
        print("Example:")
        print("  python3 visualize_nerve_tumor.py resource/tissue/neuroma_refined_uniform.msh resource/rod/clean_surface_nerve.geo")
        print("  python3 visualize_nerve_tumor.py tumor.obj nerve.geo --save-images")
        sys.exit(1)
    
    tumor_file = sys.argv[1]
    nerve_file = sys.argv[2]
    save_images = '--save-images' in sys.argv
    
    # Check files exist
    if not os.path.exists(tumor_file):
        print(f"Error: Tumor file '{tumor_file}' not found")
        sys.exit(1)
    
    if not os.path.exists(nerve_file):
        print(f"Error: Nerve file '{nerve_file}' not found")
        sys.exit(1)
    
    print("Loading tumor mesh...")
    tumor_vertices = parse_tumor_file(tumor_file, max_vertices=10000)  # Limit for performance
    
    print("Loading nerve geometry...")
    nerve_points = parse_nerve_rod_geo(nerve_file)
    
    if len(tumor_vertices) == 0 and len(nerve_points) == 0:
        print("Error: No data loaded from either file")
        sys.exit(1)
    
    # Create visualizations
    print("Creating 3D visualization...")
    fig_3d, ax_3d = create_3d_visualization(tumor_vertices, nerve_points, 
                                           "Neuroma-Nerve Assembly (3D View)")
    
    print("Creating 2D projections...")
    fig_2d, axes_2d = create_2d_projections(tumor_vertices, nerve_points,
                                           "Neuroma-Nerve Assembly (2D Projections)")
    
    # Save images if requested
    if save_images:
        fig_3d.savefig('nerve_tumor_3d.png', dpi=300, bbox_inches='tight')
        fig_2d.savefig('nerve_tumor_2d.png', dpi=300, bbox_inches='tight')
        print("Saved: nerve_tumor_3d.png, nerve_tumor_2d.png")
    
    # Check if nerve passes through tumor
    if len(tumor_vertices) > 0 and len(nerve_points) > 0:
        tumor_min = np.min(tumor_vertices, axis=0)
        tumor_max = np.max(tumor_vertices, axis=0)
        
        nerve_in_tumor = 0
        for point in nerve_points:
            if (tumor_min[0] <= point[0] <= tumor_max[0] and 
                tumor_min[1] <= point[1] <= tumor_max[1] and 
                tumor_min[2] <= point[2] <= tumor_max[2]):
                nerve_in_tumor += 1
        
        print(f"\nGeometry Analysis:")
        print(f"  Nerve points inside tumor bounding box: {nerve_in_tumor}/{len(nerve_points)} ({100*nerve_in_tumor/len(nerve_points):.1f}%)")
        
        if nerve_in_tumor > 0:
            print("  ✅ Good: Nerve passes through tumor region")
        else:
            print("  ⚠️  Warning: Nerve may not intersect tumor")
    
    print("\nVisualization complete! Close the plot windows to exit.")
    plt.show()

if __name__ == "__main__":
    main()