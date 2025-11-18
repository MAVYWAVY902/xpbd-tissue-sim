#!/usr/bin/env python3
"""
Clean Surface Nerve Generator for OBJ files
Creates a smooth nerve that lies on tumor surface using OBJ format
"""

import sys
import os
import numpy as np

def parse_obj_file(obj_file):
    """Parse OBJ file to get vertices and faces"""
    vertices = []
    faces = []
    
    try:
        with open(obj_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('v '):  # Vertex
                    parts = line.split()
                    if len(parts) >= 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        vertices.append([x, y, z])
                elif line.startswith('f '):  # Face
                    parts = line.split()
                    if len(parts) >= 4:
                        # Handle different face formats: f v1 v2 v3 or f v1/vt1 v2/vt2 v3/vt3
                        face_indices = []
                        for i in range(1, 4):  # Take first 3 vertices for triangle
                            vertex_info = parts[i].split('/')
                            vertex_index = int(vertex_info[0]) - 1  # OBJ is 1-indexed
                            face_indices.append(vertex_index)
                        faces.append(face_indices)
        
        vertices = np.array(vertices)
        faces = np.array(faces)
        
        print(f"Loaded OBJ file: {len(vertices)} vertices, {len(faces)} faces")
        return vertices, faces
        
    except Exception as e:
        print(f"Error reading OBJ file: {e}")
        return np.array([]), np.array([])

def find_surface_path_simple(bbox_min, bbox_max, surface='top', length_factor=0.7, num_points=25, offset=0.01):
    """Generate a simple path on tumor surface without complex projection"""
    
    center = (bbox_min + bbox_max) / 2
    size = bbox_max - bbox_min
    
    points = []
    
    if surface == 'top':
        # Nerve runs along Y direction on top surface
        z_surface = bbox_max[2] - offset  # Just below top surface
        start_y = center[1] - size[1] * length_factor / 2
        end_y = center[1] + size[1] * length_factor / 2
        
        for i in range(num_points):
            t = i / (num_points - 1)
            y = start_y + t * (end_y - start_y)
            # Add slight X variation for more realistic curvature
            x_offset = 0.5 * np.sin(t * np.pi) * size[0] * 0.1  # 10% width variation
            x = center[0] + x_offset
            points.append([x, y, z_surface])
            
    elif surface == 'front':
        # Nerve runs along X direction on front surface
        y_surface = bbox_max[1] - offset
        start_x = center[0] - size[0] * length_factor / 2
        end_x = center[0] + size[0] * length_factor / 2
        
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * (end_x - start_x)
            # Add slight Z variation
            z_offset = 0.3 * np.sin(t * np.pi) * size[2] * 0.1
            z = center[2] + z_offset
            points.append([x, y_surface, z])
            
    elif surface == 'side':
        # Nerve runs along Z direction on side surface  
        x_surface = bbox_max[0] - offset
        start_z = center[2] - size[2] * length_factor / 2
        end_z = center[2] + size[2] * length_factor / 2
        
        for i in range(num_points):
            t = i / (num_points - 1)
            z = start_z + t * (end_z - start_z)
            # Add slight Y variation
            y_offset = 0.3 * np.sin(t * np.pi) * size[1] * 0.1
            y = center[1] + y_offset
            points.append([x_surface, y, z])
    
    return np.array(points)

def generate_smooth_surface_nerve(vertices, faces, config):
    """Generate smooth nerve on tumor surface"""
    
    # Calculate bounding box
    bbox_min = np.min(vertices, axis=0)
    bbox_max = np.max(vertices, axis=0)
    center = (bbox_min + bbox_max) / 2
    size = bbox_max - bbox_min
    
    print(f"Tumor bounding box:")
    print(f"  Min: [{bbox_min[0]:.1f}, {bbox_min[1]:.1f}, {bbox_min[2]:.1f}]")
    print(f"  Max: [{bbox_max[0]:.1f}, {bbox_max[1]:.1f}, {bbox_max[2]:.1f}]")
    print(f"  Size: [{size[0]:.1f}, {size[1]:.1f}, {size[2]:.1f}]")
    
    # Generate simple surface path
    surface = config.get('surface', 'top')
    length_factor = config.get('length_factor', 0.7)
    num_points = config.get('num_points', 25)
    offset = config.get('offset', 0.5)  # 5mm offset from surface
    
    print(f"Generating {surface} surface nerve ({num_points} points, {length_factor*100:.0f}% length)")
    
    nerve_points = find_surface_path_simple(bbox_min, bbox_max, surface, length_factor, num_points, offset)
    
    return nerve_points

def write_nerve_geo_file(points, output_file, description):
    """Write nerve points to .geo file"""
    
    with open(output_file, 'w') as f:
        f.write(f"// {description}\n")
        f.write(f"// Clean surface nerve with {len(points)} points\n\n")
        
        # Write points with appropriate mesh size
        for i, point in enumerate(points):
            f.write(f"Point({i+1}) = {{{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}, 0.1}};\n")
        
        f.write("\n")
        
        # Write connecting lines
        for i in range(len(points) - 1):
            f.write(f"Line({i+1}) = {{{i+1}, {i+2}}};\n")
        
        f.write("\n")
        
        # Physical group
        if len(points) > 1:
            line_list = ", ".join(str(i+1) for i in range(len(points) - 1))
            f.write(f'Physical Line("nerve_edge") = {{{line_list}}};\n')

def get_nerve_configurations():
    """Available nerve configurations"""
    return {
        'top_long': {
            'description': 'Long nerve along top surface (Y direction)',
            'surface': 'top',
            'length_factor': 0.8,
            'num_points': 30,
            'offset': 0.5
        },
        'top_medium': {
            'description': 'Medium nerve along top surface', 
            'surface': 'top',
            'length_factor': 0.6,
            'num_points': 20,
            'offset': 0.3
        },
        'front_long': {
            'description': 'Long nerve along front surface (X direction)',
            'surface': 'front',
            'length_factor': 0.8,
            'num_points': 30,
            'offset': 0.5
        },
        'side_curve': {
            'description': 'Curved nerve along side surface (Z direction)',
            'surface': 'side',
            'length_factor': 0.7,
            'num_points': 25,
            'offset': 0.4
        }
    }

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 generate_obj_surface_nerve.py <tumor.obj> [config] [output.geo]")
        print("\nAvailable configurations:")
        configs = get_nerve_configurations()
        for name, config in configs.items():
            print(f"  {name}: {config['description']}")
        print(f"\nExample: python3 generate_obj_surface_nerve.py neuroma.obj top_long")
        sys.exit(1)
    
    obj_file = sys.argv[1]
    config_name = sys.argv[2] if len(sys.argv) > 2 else 'top_long'
    output_file = sys.argv[3] if len(sys.argv) > 3 else f"clean_nerve_{config_name}.geo"
    
    # Check input
    if not os.path.exists(obj_file):
        print(f"Error: File '{obj_file}' not found")
        sys.exit(1)
    
    # Get configuration
    configs = get_nerve_configurations()
    if config_name not in configs:
        print(f"Error: Unknown configuration '{config_name}'")
        print(f"Available: {list(configs.keys())}")
        sys.exit(1)
    
    config = configs[config_name]
    print(f"Using: {config['description']}")
    
    # Load OBJ file
    print(f"Loading {obj_file}...")
    vertices, faces = parse_obj_file(obj_file)
    
    if len(vertices) == 0:
        print("Error: No vertices found")
        sys.exit(1)
    
    # Generate nerve
    nerve_points = generate_smooth_surface_nerve(vertices, faces, config)
    
    if len(nerve_points) == 0:
        print("Error: Failed to generate nerve points")
        sys.exit(1)
    
    # Write output
    write_nerve_geo_file(nerve_points, output_file, config['description'])
    
    print(f"\n✅ Generated clean nerve: {output_file}")
    print(f"   Points: {len(nerve_points)}")
    print(f"   Length: ~{config['length_factor']*100:.0f}% of tumor size")
    
    # Show next steps
    print(f"\nNext steps:")
    print(f"1. gmsh -1 {output_file} -o {output_file[:-4]}.msh")
    print(f"2. python3 scripts/visualize_nerve_tumor.py {obj_file} {output_file}")

if __name__ == "__main__":
    main()