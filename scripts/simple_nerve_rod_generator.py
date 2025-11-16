#!/usr/bin/env python3
"""
Simple Nerve Rod Generator (No dependencies)
Reads basic mesh info from Gmsh .msh files and generates matching rod
"""

import sys
import os
import re

def parse_gmsh_msh_simple(msh_file):
    """Parse Gmsh .msh file to extract vertex coordinates (supports both v2.2 and v4.1 formats)"""
    vertices = []
    
    try:
        with open(msh_file, 'r') as f:
            lines = f.readlines()
        
        # Detect Gmsh version
        version = "2.2"  # default
        for line in lines:
            if line.strip().startswith('$MeshFormat'):
                continue
            elif line.strip() and not line.startswith('$'):
                version_info = line.strip().split()
                if len(version_info) > 0:
                    version = version_info[0]
                    break
        
        print(f"Detected Gmsh format version: {version}")
        
        # Find $Nodes section
        node_section = False
        for i, line in enumerate(lines):
            if line.strip() == '$Nodes':
                node_section = True
                print(f"Found $Nodes section at line {i+1}")
                
                if version.startswith('4'):
                    # Gmsh 4.x format parsing
                    # Line after $Nodes: numEntityBlocks numNodes minNodeTag maxNodeTag
                    if i + 1 < len(lines):
                        header_parts = lines[i + 1].strip().split()
                        if len(header_parts) >= 2:
                            num_nodes = int(header_parts[1])
                            print(f"Gmsh 4.x format: {num_nodes} total nodes")
                            
                            # Parse entity blocks
                            line_idx = i + 2
                            nodes_read = 0
                            
                            while line_idx < len(lines) and nodes_read < num_nodes:
                                line_content = lines[line_idx].strip()
                                if line_content == '$EndNodes':
                                    break
                                
                                # Try to parse as entity block header: entityDim entityTag parametric numNodesInBlock
                                entity_parts = line_content.split()
                                if len(entity_parts) == 4 and entity_parts[0].isdigit():
                                    entity_dim, entity_tag, parametric, num_nodes_in_block = map(int, entity_parts)
                                    line_idx += 1
                                    
                                    # Read node tags first (if any)
                                    node_tags = []
                                    for tag_line_offset in range(num_nodes_in_block):
                                        if line_idx + tag_line_offset < len(lines):
                                            tag_line = lines[line_idx + tag_line_offset].strip()
                                            if tag_line.isdigit():
                                                node_tags.append(int(tag_line))
                                            else:
                                                break
                                    
                                    # Skip to coordinates (after node tags)
                                    line_idx += len(node_tags)
                                    
                                    # Read coordinates
                                    for coord_offset in range(num_nodes_in_block):
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
                    # Gmsh 2.x format parsing (original code)
                    if i + 1 < len(lines):
                        try:
                            num_nodes = int(lines[i + 1].strip())
                            print(f"Gmsh 2.x format: {num_nodes} nodes")
                            
                            # Read node coordinates
                            for j in range(i + 2, min(i + 2 + num_nodes, len(lines))):
                                parts = lines[j].strip().split()
                                if len(parts) >= 4:  # node_id x y z
                                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                                    vertices.append([x, y, z])
                            break
                        except ValueError:
                            continue
            elif line.strip() == '$EndNodes':
                break
        
        if not vertices:
            print("Warning: No vertices found in mesh file")
            print("Debug: First 10 lines of file:")
            for i, line in enumerate(lines[:10]):
                print(f"  {i+1}: {line.strip()}")
            return None
        
        # Calculate bounding box
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        zs = [v[2] for v in vertices]
        
        bbox_min = [min(xs), min(ys), min(zs)]
        bbox_max = [max(xs), max(ys), max(zs)]
        bbox_center = [(bbox_min[i] + bbox_max[i]) / 2 for i in range(3)]
        bbox_size = [bbox_max[i] - bbox_min[i] for i in range(3)]
        
        print(f"Tumor mesh analysis:")
        print(f"  Vertices: {len(vertices)}")
        print(f"  Bounding box min: [{bbox_min[0]:.3f}, {bbox_min[1]:.3f}, {bbox_min[2]:.3f}]")
        print(f"  Bounding box max: [{bbox_max[0]:.3f}, {bbox_max[1]:.3f}, {bbox_max[2]:.3f}]")
        print(f"  Center: [{bbox_center[0]:.3f}, {bbox_center[1]:.3f}, {bbox_center[2]:.3f}]")
        print(f"  Size: [{bbox_size[0]:.3f}, {bbox_size[1]:.3f}, {bbox_size[2]:.3f}]")
        
        return {
            'bbox_min': bbox_min,
            'bbox_max': bbox_max,
            'bbox_center': bbox_center,
            'bbox_size': bbox_size,
            'num_vertices': len(vertices)
        }
        
    except Exception as e:
        print(f"Error parsing mesh file: {e}")
        import traceback
        traceback.print_exc()
        return None

def generate_rod_configurations():
    """Return different rod configuration templates"""
    return {
        'through_x': {
            'description': 'Rod passes through tumor in X direction',
            'direction': 'x',
            'num_points': 25,
            'length_factor': 1.2,
            'offset': [0, 0, 0]
        },
        'through_y': {
            'description': 'Rod passes through tumor in Y direction', 
            'direction': 'y',
            'num_points': 25,
            'length_factor': 1.2,
            'offset': [0, 0, 0]
        },
        'through_z': {
            'description': 'Rod passes through tumor in Z direction',
            'direction': 'z', 
            'num_points': 25,
            'length_factor': 1.2,
            'offset': [0, 0, 0]
        },
        'surface_top': {
            'description': 'Rod lies on top surface of tumor',
            'surface': 'top',
            'direction': 'y',  # Rod direction along surface
            'num_points': 20,
            'length_factor': 0.8,
            'surface_offset': 0.01
        },
        'surface_front': {
            'description': 'Rod lies on front surface of tumor',
            'surface': 'front', 
            'direction': 'x',
            'num_points': 20,
            'length_factor': 0.8,
            'surface_offset': 0.01
        }
    }

def generate_rod_through(tumor_info, config):
    """Generate rod that passes through tumor center"""
    center = tumor_info['bbox_center']
    size = tumor_info['bbox_size']
    
    direction = config['direction']
    num_points = config['num_points']
    length_factor = config['length_factor']
    offset = config['offset']
    
    # Calculate rod direction and length
    if direction == 'x':
        rod_length = size[0] * length_factor
        dir_vec = [1, 0, 0]
    elif direction == 'y':
        rod_length = size[1] * length_factor
        dir_vec = [0, 1, 0]
    else:  # 'z'
        rod_length = size[2] * length_factor
        dir_vec = [0, 0, 1]
    
    # Generate points
    rod_center = [center[i] + offset[i] for i in range(3)]
    start = [rod_center[i] - dir_vec[i] * rod_length / 2 for i in range(3)]
    end = [rod_center[i] + dir_vec[i] * rod_length / 2 for i in range(3)]
    
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        point = [start[j] + t * (end[j] - start[j]) for j in range(3)]
        points.append(point)
    
    return points

def generate_rod_on_surface(tumor_info, config):
    """Generate rod that lies on tumor surface"""
    center = tumor_info['bbox_center']
    size = tumor_info['bbox_size']
    
    surface = config['surface']
    direction = config['direction'] 
    num_points = config['num_points']
    length_factor = config['length_factor']
    surface_offset = config['surface_offset']
    
    points = []
    
    if surface == 'top':
        # Rod on top surface, running in specified direction
        surface_z = tumor_info['bbox_max'][2] + surface_offset
        if direction == 'y':
            start_y = center[1] - size[1] * length_factor / 2
            end_y = center[1] + size[1] * length_factor / 2
            for i in range(num_points):
                t = i / (num_points - 1)
                y = start_y + t * (end_y - start_y)
                points.append([center[0], y, surface_z])
        else:  # direction == 'x'
            start_x = center[0] - size[0] * length_factor / 2
            end_x = center[0] + size[0] * length_factor / 2
            for i in range(num_points):
                t = i / (num_points - 1)
                x = start_x + t * (end_x - start_x)
                points.append([x, center[1], surface_z])
                
    elif surface == 'front':
        # Rod on front surface
        surface_y = tumor_info['bbox_max'][1] + surface_offset
        if direction == 'x':
            start_x = center[0] - size[0] * length_factor / 2
            end_x = center[0] + size[0] * length_factor / 2
            for i in range(num_points):
                t = i / (num_points - 1)
                x = start_x + t * (end_x - start_x)
                points.append([x, surface_y, center[2]])
    
    return points

def write_gmsh_geo_file(points, output_file, config_name):
    """Write rod points to Gmsh .geo file"""
    
    with open(output_file, 'w') as f:
        f.write(f"// Auto-generated nerve rod: {config_name}\n")
        f.write(f"// Points: {len(points)}\n\n")
        
        # Write points
        for i, point in enumerate(points):
            f.write(f"Point({i+1}) = {{{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}, 1e-3}};\n")
        
        f.write("\n")
        
        # Write lines
        for i in range(len(points) - 1):
            f.write(f"Line({i+1}) = {{{i+1}, {i+2}}};\n")
        
        f.write("\n")
        
        # Physical group
        if len(points) > 1:
            line_list = ", ".join(str(i+1) for i in range(len(points) - 1))
            f.write(f'Physical Line("nerve_edge") = {{{line_list}}};\n')

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 generate_nerve_rod.py <tumor.msh> [config_name] [output.geo]")
        print("\nAvailable configurations:")
        configs = generate_rod_configurations()
        for name, config in configs.items():
            print(f"  {name}: {config['description']}")
        print(f"\nExample: python3 generate_nerve_rod.py cube8_nerve.msh through_y")
        sys.exit(1)
    
    tumor_file = sys.argv[1]
    config_name = sys.argv[2] if len(sys.argv) > 2 else 'through_y'
    output_file = sys.argv[3] if len(sys.argv) > 3 else f"nerve_rod_{config_name}.geo"
    
    # Check input file
    if not os.path.exists(tumor_file):
        print(f"Error: File '{tumor_file}' not found")
        sys.exit(1)
    
    # Get configuration
    configs = generate_rod_configurations()
    if config_name not in configs:
        print(f"Error: Unknown configuration '{config_name}'")
        print(f"Available: {list(configs.keys())}")
        sys.exit(1)
    
    config = configs[config_name]
    print(f"Using configuration: {config['description']}")
    
    # Parse tumor mesh
    tumor_info = parse_gmsh_msh_simple(tumor_file)
    if tumor_info is None:
        sys.exit(1)
    
    # Generate rod points
    if 'surface' in config:
        points = generate_rod_on_surface(tumor_info, config)
    else:
        points = generate_rod_through(tumor_info, config)
    
    if not points:
        print("Error: Failed to generate rod points")
        sys.exit(1)
    
    # Write output
    write_gmsh_geo_file(points, output_file, config_name)
    
    print(f"\nGenerated rod with {len(points)} points")
    print(f"Output file: {output_file}")
    print(f"\nNext steps:")
    print(f"1. Generate mesh: gmsh -1 {output_file} -o {output_file[:-4]}.msh")
    print(f"2. Update config to use: {output_file[:-4]}.msh")
    print(f"3. Set nerve position: [0, 0, 0] (pre-positioned)")

if __name__ == "__main__":
    main()