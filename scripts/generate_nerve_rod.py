#!/usr/bin/env python3
"""
Nerve Rod Generator for Tumor Mesh
Generates a 1D rod mesh that passes through or lies on tumor surface

Dependencies: numpy, meshio (pip install meshio)
"""

import numpy as np
import sys
import os

def read_tumor_mesh(msh_file):
    """Read tumor mesh and extract basic info"""
    try:
        import meshio
        mesh = meshio.read(msh_file)
        
        # Get all vertices
        vertices = mesh.points
        
        # Calculate bounding box
        bbox_min = np.min(vertices, axis=0)
        bbox_max = np.max(vertices, axis=0)
        bbox_center = (bbox_min + bbox_max) / 2
        bbox_size = bbox_max - bbox_min
        
        print(f"Tumor mesh info:")
        print(f"  Vertices: {len(vertices)}")
        print(f"  Bounding box min: [{bbox_min[0]:.3f}, {bbox_min[1]:.3f}, {bbox_min[2]:.3f}]")
        print(f"  Bounding box max: [{bbox_max[0]:.3f}, {bbox_max[1]:.3f}, {bbox_max[2]:.3f}]")
        print(f"  Center: [{bbox_center[0]:.3f}, {bbox_center[1]:.3f}, {bbox_center[2]:.3f}]")
        print(f"  Size: [{bbox_size[0]:.3f}, {bbox_size[1]:.3f}, {bbox_size[2]:.3f}]")
        
        return {
            'vertices': vertices,
            'bbox_min': bbox_min,
            'bbox_max': bbox_max,
            'bbox_center': bbox_center,
            'bbox_size': bbox_size,
            'mesh': mesh
        }
        
    except ImportError:
        print("Error: meshio not installed. Install with: pip install meshio")
        return None
    except Exception as e:
        print(f"Error reading mesh file {msh_file}: {e}")
        return None

def generate_rod_through_tumor(tumor_info, rod_config):
    """
    Generate 1D rod that passes through tumor
    
    Args:
        tumor_info: dict from read_tumor_mesh()
        rod_config: dict with rod parameters
    """
    center = tumor_info['bbox_center']
    size = tumor_info['bbox_size']
    
    # Rod configuration
    direction = rod_config.get('direction', 'z')  # 'x', 'y', or 'z'
    num_points = rod_config.get('num_points', 20)
    length_factor = rod_config.get('length_factor', 1.2)  # 120% of tumor size
    offset_from_center = rod_config.get('offset', [0, 0, 0])
    
    # Calculate rod length and direction vector
    if direction == 'x':
        rod_length = size[0] * length_factor
        dir_vec = np.array([1, 0, 0])
    elif direction == 'y':
        rod_length = size[1] * length_factor
        dir_vec = np.array([0, 1, 0])
    else:  # 'z'
        rod_length = size[2] * length_factor
        dir_vec = np.array([0, 0, 1])
    
    # Generate rod points
    rod_center = center + np.array(offset_from_center)
    start_point = rod_center - dir_vec * rod_length / 2
    end_point = rod_center + dir_vec * rod_length / 2
    
    # Create evenly spaced points
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)  # Parameter from 0 to 1
        point = start_point + t * (end_point - start_point)
        points.append(point)
    
    print(f"Generated rod:")
    print(f"  Direction: {direction}")
    print(f"  Points: {num_points}")
    print(f"  Length: {rod_length:.3f}")
    print(f"  Start: [{start_point[0]:.3f}, {start_point[1]:.3f}, {start_point[2]:.3f}]")
    print(f"  End: [{end_point[0]:.3f}, {end_point[1]:.3f}, {end_point[2]:.3f}]")
    
    return np.array(points)

def generate_rod_on_tumor_surface(tumor_info, rod_config):
    """
    Generate 1D rod that lies on tumor surface
    """
    center = tumor_info['bbox_center']
    size = tumor_info['bbox_size']
    
    # Rod configuration
    surface = rod_config.get('surface', 'top')  # 'top', 'bottom', 'front', 'back', 'left', 'right'
    num_points = rod_config.get('num_points', 20)
    length_factor = rod_config.get('length_factor', 0.8)  # 80% of tumor surface
    surface_offset = rod_config.get('surface_offset', 0.01)  # Small offset above surface
    
    # Define surface positions and rod directions
    if surface == 'top':
        surface_z = tumor_info['bbox_max'][2] + surface_offset
        # Rod runs along Y direction on top surface
        start_y = center[1] - size[1] * length_factor / 2
        end_y = center[1] + size[1] * length_factor / 2
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            y = start_y + t * (end_y - start_y)
            points.append([center[0], y, surface_z])
            
    elif surface == 'front':
        surface_y = tumor_info['bbox_max'][1] + surface_offset
        # Rod runs along X direction on front surface
        start_x = center[0] - size[0] * length_factor / 2
        end_x = center[0] + size[0] * length_factor / 2
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (end_x - start_x)
            points.append([x, center[1], surface_y])
    
    # Add more surface options as needed...
    else:
        # Default to top surface
        return generate_rod_on_tumor_surface(tumor_info, {**rod_config, 'surface': 'top'})
    
    print(f"Generated rod on {surface} surface:")
    print(f"  Points: {num_points}")
    print(f"  Surface offset: {surface_offset}")
    
    return np.array(points)

def write_gmsh_geo_file(points, output_file, physical_name="nerve_edge"):
    """Write rod points to Gmsh .geo file"""
    
    with open(output_file, 'w') as f:
        f.write("// Auto-generated nerve rod for tumor adhesion testing\n")
        f.write(f"// Generated by nerve rod generator\n")
        f.write(f"// Points: {len(points)}\n\n")
        
        # Write points
        for i, point in enumerate(points):
            f.write(f"Point({i+1}) = {{{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}, 1e-3}};\n")
        
        f.write("\n")
        
        # Write lines connecting consecutive points
        for i in range(len(points) - 1):
            f.write(f"Line({i+1}) = {{{i+1}, {i+2}}};\n")
        
        f.write("\n")
        
        # Write physical group
        if len(points) > 1:
            line_list = ", ".join(str(i+1) for i in range(len(points) - 1))
            f.write(f'Physical Line("{physical_name}") = {{{line_list}}};\n')

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 generate_nerve_rod.py <tumor.msh> <output_rod.geo> [config]")
        print("\nExamples:")
        print("  python3 generate_nerve_rod.py tumor.msh nerve_rod.geo")
        print("  python3 generate_nerve_rod.py tumor.msh nerve_through.geo --through")
        print("  python3 generate_nerve_rod.py tumor.msh nerve_surface.geo --surface")
        sys.exit(1)
    
    tumor_file = sys.argv[1]
    output_file = sys.argv[2]
    mode = 'through'  # default mode
    
    # Parse command line options
    if len(sys.argv) > 3:
        if '--surface' in sys.argv:
            mode = 'surface'
        elif '--through' in sys.argv:
            mode = 'through'
    
    # Check if tumor file exists
    if not os.path.exists(tumor_file):
        print(f"Error: Tumor mesh file '{tumor_file}' not found")
        sys.exit(1)
    
    # Read tumor mesh
    print(f"Reading tumor mesh: {tumor_file}")
    tumor_info = read_tumor_mesh(tumor_file)
    if tumor_info is None:
        sys.exit(1)
    
    # Generate rod based on mode
    if mode == 'surface':
        rod_config = {
            'surface': 'top',
            'num_points': 25,
            'length_factor': 0.8,
            'surface_offset': 0.005  # 5mm above surface
        }
        points = generate_rod_on_tumor_surface(tumor_info, rod_config)
    else:  # through mode
        rod_config = {
            'direction': 'y',  # Rod passes through tumor in Y direction
            'num_points': 30,
            'length_factor': 1.3,  # 30% longer than tumor
            'offset': [0, 0, 0.1]  # Slight Z offset
        }
        points = generate_rod_through_tumor(tumor_info, rod_config)
    
    # Write output file
    print(f"\nWriting rod geometry: {output_file}")
    write_gmsh_geo_file(points, output_file)
    
    # Instructions for next steps
    print(f"\nNext steps:")
    print(f"1. Generate mesh: gmsh -1 {output_file} -o {output_file[:-4]}.msh")
    print(f"2. Update config file to use: {output_file[:-4]}.msh")
    print(f"3. Set nerve position: [0, 0, 0] (rod already positioned)")

if __name__ == "__main__":
    main()