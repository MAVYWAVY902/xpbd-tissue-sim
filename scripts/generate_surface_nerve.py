#!/usr/bin/env python3
"""
Surface-Following Nerve Generator
Creates a nerve rod that follows the tumor surface curvature for realistic adhesion testing

Dependencies: numpy, scipy (for spatial operations)
"""

import sys
import os
import numpy as np
from collections import defaultdict

def parse_gmsh_surface_mesh(msh_file):
    """Parse Gmsh .msh file to extract vertices and surface triangles"""
    vertices = []
    triangles = []
    
    try:
        with open(msh_file, 'r') as f:
            lines = f.readlines()
        
        # Parse vertices (same as before)
        version = "2.2"
        for line in lines:
            if line.strip().startswith('$MeshFormat'):
                continue
            elif line.strip() and not line.startswith('$'):
                version_info = line.strip().split()
                if len(version_info) > 0:
                    version = version_info[0]
                    break
        
        print(f"Parsing mesh format version: {version}")
        
        # Extract vertices
        for i, line in enumerate(lines):
            if line.strip() == '$Nodes':
                if version.startswith('4'):
                    header_parts = lines[i + 1].strip().split()
                    if len(header_parts) >= 2:
                        total_nodes = int(header_parts[1])
                        print(f"Loading {total_nodes} vertices...")
                        
                        line_idx = i + 2
                        nodes_read = 0
                        node_id_map = {}  # Map node_id to index
                        
                        while line_idx < len(lines) and nodes_read < total_nodes:
                            line_content = lines[line_idx].strip()
                            if line_content == '$EndNodes':
                                break
                            
                            entity_parts = line_content.split()
                            if len(entity_parts) == 4 and entity_parts[0].isdigit():
                                entity_dim, entity_tag, parametric, num_nodes_in_block = map(int, entity_parts)
                                line_idx += 1
                                
                                # Read node IDs
                                node_ids = []
                                for tag_offset in range(num_nodes_in_block):
                                    if line_idx + tag_offset < len(lines):
                                        tag_line = lines[line_idx + tag_offset].strip()
                                        if tag_line.isdigit():
                                            node_ids.append(int(tag_line))
                                        else:
                                            break
                                line_idx += len(node_ids)
                                
                                # Read coordinates
                                for coord_offset in range(num_nodes_in_block):
                                    coord_line_idx = line_idx + coord_offset
                                    if coord_line_idx < len(lines):
                                        coord_parts = lines[coord_line_idx].strip().split()
                                        if len(coord_parts) >= 3:
                                            try:
                                                x, y, z = float(coord_parts[0]), float(coord_parts[1]), float(coord_parts[2])
                                                vertices.append([x, y, z])
                                                if coord_offset < len(node_ids):
                                                    node_id_map[node_ids[coord_offset]] = len(vertices) - 1
                                                nodes_read += 1
                                            except ValueError:
                                                continue
                                
                                line_idx += num_nodes_in_block
                            else:
                                line_idx += 1
                        break
                break
        
        # Extract surface triangles
        print("Extracting surface triangles...")
        for i, line in enumerate(lines):
            if line.strip() == '$Elements':
                if version.startswith('4'):
                    header_parts = lines[i + 1].strip().split()
                    if len(header_parts) >= 2:
                        total_elements = int(header_parts[1])
                        line_idx = i + 2
                        
                        while line_idx < len(lines):
                            line_content = lines[line_idx].strip()
                            if line_content == '$EndElements':
                                break
                            
                            # Parse element block header
                            elem_parts = line_content.split()
                            if len(elem_parts) == 4 and elem_parts[0].isdigit():
                                entity_dim, entity_tag, element_type, num_elements_in_block = map(int, elem_parts)
                                line_idx += 1
                                
                                # Only process surface elements (triangles: type 2)
                                if element_type == 2:  # Triangle
                                    for elem_offset in range(num_elements_in_block):
                                        elem_line_idx = line_idx + elem_offset
                                        if elem_line_idx < len(lines):
                                            elem_line_parts = lines[elem_line_idx].strip().split()
                                            if len(elem_line_parts) >= 4:  # elem_id + 3 node_ids
                                                try:
                                                    node_ids = [int(elem_line_parts[j]) for j in range(1, 4)]
                                                    # Convert to 0-based indices
                                                    indices = [node_id_map.get(nid, -1) for nid in node_ids]
                                                    if all(idx >= 0 for idx in indices):
                                                        triangles.append(indices)
                                                except (ValueError, IndexError):
                                                    continue
                                
                                line_idx += num_elements_in_block
                            else:
                                line_idx += 1
                        break
                break
        
        vertices = np.array(vertices)
        triangles = np.array(triangles)
        
        print(f"Loaded: {len(vertices)} vertices, {len(triangles)} triangles")
        
        return vertices, triangles
        
    except Exception as e:
        print(f"Error parsing mesh: {e}")
        import traceback
        traceback.print_exc()
        return np.array([]), np.array([])

def find_surface_path(vertices, triangles, start_point, end_point, num_segments=20):
    """
    Find a path along the tumor surface from start to end point
    Uses simple projection and surface following
    """
    
    if len(triangles) == 0:
        print("Warning: No triangles found, using simple interpolation")
        # Fallback: simple linear interpolation
        path_points = []
        for i in range(num_segments + 1):
            t = i / num_segments
            point = start_point + t * (end_point - start_point)
            path_points.append(point)
        return np.array(path_points)
    
    # Create path points by projecting onto surface
    path_points = []
    
    for i in range(num_segments + 1):
        t = i / num_segments
        # Linear interpolation between start and end
        current_point = start_point + t * (end_point - start_point)
        
        # Project onto nearest surface triangle
        projected_point = project_point_to_surface(current_point, vertices, triangles)
        path_points.append(projected_point)
    
    return np.array(path_points)

def project_point_to_surface(point, vertices, triangles):
    """Project a point onto the nearest triangle surface"""
    
    min_distance = float('inf')
    best_projection = point.copy()
    
    for tri_idx in range(min(len(triangles), 1000)):  # Limit for performance
        triangle = triangles[tri_idx]
        v0, v1, v2 = vertices[triangle]
        
        # Project point onto triangle plane
        projected = project_point_to_triangle(point, v0, v1, v2)
        distance = np.linalg.norm(point - projected)
        
        if distance < min_distance:
            min_distance = distance
            best_projection = projected
    
    return best_projection

def project_point_to_triangle(point, v0, v1, v2):
    """Project point onto triangle surface using barycentric coordinates"""
    
    # Compute triangle edges and normal
    edge1 = v1 - v0
    edge2 = v2 - v0
    normal = np.cross(edge1, edge2)
    
    if np.linalg.norm(normal) < 1e-10:  # Degenerate triangle
        return v0  # Return first vertex as fallback
    
    normal = normal / np.linalg.norm(normal)
    
    # Project point onto triangle plane
    to_point = point - v0
    distance_to_plane = np.dot(to_point, normal)
    projected_on_plane = point - distance_to_plane * normal
    
    # Check if projection is inside triangle using barycentric coordinates
    v_to_proj = projected_on_plane - v0
    
    # Solve barycentric coordinates
    dot00 = np.dot(edge2, edge2)
    dot01 = np.dot(edge2, edge1)
    dot02 = np.dot(edge2, v_to_proj)
    dot11 = np.dot(edge1, edge1)
    dot12 = np.dot(edge1, v_to_proj)
    
    inv_denom = 1 / (dot00 * dot11 - dot01 * dot01 + 1e-10)
    u = (dot11 * dot02 - dot01 * dot12) * inv_denom
    v = (dot00 * dot12 - dot01 * dot02) * inv_denom
    
    # Clamp barycentric coordinates to triangle
    u = max(0, min(1, u))
    v = max(0, min(1, v))
    if u + v > 1:
        u = u / (u + v)
        v = v / (u + v)
    
    w = 1 - u - v
    
    # Return point on triangle
    return w * v0 + u * v1 + v * v2

def generate_surface_nerve_configurations():
    """Different surface nerve configurations"""
    return {
        'top_surface_curve': {
            'description': 'Long nerve curves along top surface of tumor',
            'surface': 'top',
            'curve_type': 'gentle',
            'num_segments': 40  # More segments for longer nerve
        },
        'side_surface_wrap': {
            'description': 'Long nerve wraps around side surface',
            'surface': 'side',
            'curve_type': 'wrap',
            'num_segments': 50  # Even more for side wrap
        },
        'front_surface_line': {
            'description': 'Long nerve follows front surface contour',
            'surface': 'front',
            'curve_type': 'straight',
            'num_segments': 35
        },
        'diagonal_surface': {
            'description': 'Very long nerve diagonally across tumor surface',
            'surface': 'diagonal',
            'curve_type': 'straight',
            'num_segments': 60  # Longest nerve
        }
    }

def generate_surface_following_nerve(vertices, triangles, tumor_bbox, config):
    """Generate nerve that follows tumor surface"""
    
    bbox_min, bbox_max = tumor_bbox
    bbox_center = (bbox_min + bbox_max) / 2
    bbox_size = bbox_max - bbox_min
    
    surface = config.get('surface', 'top')
    curve_type = config.get('curve_type', 'gentle')
    num_segments = config.get('num_segments', 25)
    
    print(f"Generating {surface} surface nerve with {curve_type} curvature...")
    
    # Define start and end points based on surface - MUCH LONGER SPANS
    if surface == 'top':
        # Start and end on top surface, spanning FULL Y direction
        surface_z = bbox_max[2] - 1.0  # 1m below top surface to ensure it's on surface
        start_point = np.array([bbox_center[0], bbox_min[1] + bbox_size[1] * 0.05, surface_z])  # 5% from edge
        end_point = np.array([bbox_center[0], bbox_max[1] - bbox_size[1] * 0.05, surface_z])    # 95% span
        
        print(f"Top surface nerve: Y span from {start_point[1]:.1f} to {end_point[1]:.1f} (length: {np.linalg.norm(end_point - start_point):.1f}cm)")
        
    elif surface == 'front':
        # Start and end on front surface, spanning FULL X direction
        surface_y = bbox_max[1] - 1.0  # 1m inside from front
        start_point = np.array([bbox_min[0] + bbox_size[0] * 0.05, surface_y, bbox_center[2]])  # 5% from edge
        end_point = np.array([bbox_max[0] - bbox_size[0] * 0.05, surface_y, bbox_center[2]])    # 95% span
        
        print(f"Front surface nerve: X span from {start_point[0]:.1f} to {end_point[0]:.1f} (length: {np.linalg.norm(end_point - start_point):.1f}cm)")
        
    elif surface == 'side':
        # Wrap around side surface - FULL Z direction
        surface_x = bbox_max[0] - 1.0  # 1m inside from side
        start_point = np.array([surface_x, bbox_center[1], bbox_min[2] + bbox_size[2] * 0.05])  # 5% from bottom
        end_point = np.array([surface_x, bbox_center[1], bbox_max[2] - bbox_size[2] * 0.05])    # 95% to top
        
        print(f"Side surface nerve: Z span from {start_point[2]:.1f} to {end_point[2]:.1f} (length: {np.linalg.norm(end_point - start_point):.1f}cm)")
    
    else:
        # Default to diagonal across tumor (VERY LONG)
        start_point = np.array([bbox_min[0] + bbox_size[0] * 0.1, bbox_min[1] + bbox_size[1] * 0.1, bbox_max[2] - 0.5])
        end_point = np.array([bbox_max[0] - bbox_size[0] * 0.1, bbox_max[1] - bbox_size[1] * 0.1, bbox_max[2] - 0.5])
        
        print(f"Diagonal surface nerve: length {np.linalg.norm(end_point - start_point):.1f}cm")
    
    print(f"Initial path: {start_point} -> {end_point}")
    
    # Generate longer intermediate points for better surface following
    intermediate_points = []
    for i in range(num_segments + 1):
        t = i / num_segments
        # Linear interpolation between start and end
        current_point = start_point + t * (end_point - start_point)
        intermediate_points.append(current_point)
    
    # Project ALL points onto surface (not just endpoints)
    surface_points = []
    for i, point in enumerate(intermediate_points):
        projected = project_point_to_surface(point, vertices, triangles)
        
        # Add some variation to avoid identical projections
        if i > 0 and np.linalg.norm(projected - surface_points[-1]) < 0.1:  # Too close to previous
            # Nudge point slightly to avoid clustering
            direction = end_point - start_point
            direction = direction / np.linalg.norm(direction)
            projected = projected + direction * 0.2 * i  # Progressive offset
        
        surface_points.append(projected)
        
        if i % 5 == 0:  # Debug every 5th point
            print(f"  Point {i}: projected to [{projected[0]:.1f}, {projected[1]:.1f}, {projected[2]:.1f}]")
    
    surface_points = np.array(surface_points)
    
    # Add small offset outward from surface (for adhesion gap) 
    offset_points = []
    for i, point in enumerate(surface_points):
        # Find surface normal and offset outward
        normal = estimate_surface_normal_at_point(point, vertices, triangles)
        offset_point = point + normal * 0.01  # 1cm offset for better visibility
        offset_points.append(offset_point)
        
        if i % 5 == 0:  # Debug
            print(f"  Offset {i}: [{offset_point[0]:.1f}, {offset_point[1]:.1f}, {offset_point[2]:.1f}] (normal: {normal})")
    
    offset_points = np.array(offset_points)
    
    # Verify the nerve is actually long
    total_length = 0
    for i in range(1, len(offset_points)):
        segment_length = np.linalg.norm(offset_points[i] - offset_points[i-1])
        total_length += segment_length
    
    print(f"Generated nerve total length: {total_length:.1f}cm")
    
    if total_length < 5.0:  # Less than 5cm is too short
        print(f"WARNING: Nerve is very short ({total_length:.1f}cm). Expected >10cm for this tumor size.")
    
    return offset_points

def estimate_surface_normal_at_point(point, vertices, triangles):
    """Estimate surface normal at a given point"""
    
    if len(triangles) == 0:
        return np.array([0, 0, 1])  # Default upward normal
    
    # Find closest triangle
    min_distance = float('inf')
    closest_triangle = None
    
    for tri_idx in range(min(len(triangles), 500)):  # Limit for performance
        triangle = triangles[tri_idx]
        v0, v1, v2 = vertices[triangle]
        center = (v0 + v1 + v2) / 3
        distance = np.linalg.norm(point - center)
        
        if distance < min_distance:
            min_distance = distance
            closest_triangle = triangle
    
    if closest_triangle is not None:
        # Compute triangle normal
        v0, v1, v2 = vertices[closest_triangle]
        edge1 = v1 - v0
        edge2 = v2 - v0
        normal = np.cross(edge1, edge2)
        
        if np.linalg.norm(normal) > 1e-10:
            return normal / np.linalg.norm(normal)
    
    return np.array([0, 0, 1])  # Default

def write_surface_nerve_geo(points, output_file, config_name):
    """Write surface nerve points to Gmsh .geo file"""
    
    with open(output_file, 'w') as f:
        f.write(f"// Surface-following nerve: {config_name}\n")
        f.write(f"// Generated nerve that follows tumor surface curvature\n")
        f.write(f"// Points: {len(points)}\n\n")
        
        # Write points
        for i, point in enumerate(points):
            f.write(f"Point({i+1}) = {{{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}, 5e-4}};\n")
        
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
        print("Usage: python3 generate_surface_nerve.py <tumor.msh> [config_name] [output.geo]")
        print("\nAvailable configurations:")
        configs = generate_surface_nerve_configurations()
        for name, config in configs.items():
            print(f"  {name}: {config['description']}")
        print(f"\nExample: python3 generate_surface_nerve.py neuroma.msh top_surface_curve")
        sys.exit(1)
    
    tumor_file = sys.argv[1]
    config_name = sys.argv[2] if len(sys.argv) > 2 else 'top_surface_curve'
    output_file = sys.argv[3] if len(sys.argv) > 3 else f"surface_nerve_{config_name}.geo"
    
    # Check input file
    if not os.path.exists(tumor_file):
        print(f"Error: File '{tumor_file}' not found")
        sys.exit(1)
    
    # Get configuration
    configs = generate_surface_nerve_configurations()
    if config_name not in configs:
        print(f"Error: Unknown configuration '{config_name}'")
        print(f"Available: {list(configs.keys())}")
        sys.exit(1)
    
    config = configs[config_name]
    print(f"Using configuration: {config['description']}")
    
    # Parse tumor mesh
    print("Loading tumor mesh...")
    vertices, triangles = parse_gmsh_surface_mesh(tumor_file)
    
    if len(vertices) == 0:
        print("Error: No vertices found in tumor mesh")
        sys.exit(1)
    
    # Calculate bounding box
    bbox_min = np.min(vertices, axis=0)
    bbox_max = np.max(vertices, axis=0)
    bbox_center = (bbox_min + bbox_max) / 2
    bbox_size = bbox_max - bbox_min
    
    print(f"Tumor bounding box:")
    print(f"  Min: [{bbox_min[0]:.1f}, {bbox_min[1]:.1f}, {bbox_min[2]:.1f}]")
    print(f"  Max: [{bbox_max[0]:.1f}, {bbox_max[1]:.1f}, {bbox_max[2]:.1f}]")
    print(f"  Size: [{bbox_size[0]:.1f}, {bbox_size[1]:.1f}, {bbox_size[2]:.1f}]")
    
    # Generate surface-following nerve
    nerve_points = generate_surface_following_nerve(vertices, triangles, (bbox_min, bbox_max), config)
    
    if len(nerve_points) == 0:
        print("Error: Failed to generate surface nerve points")
        sys.exit(1)
    
    # Write output
    write_surface_nerve_geo(nerve_points, output_file, config_name)
    
    print(f"\nGenerated surface nerve with {len(nerve_points)} points")
    print(f"Output file: {output_file}")
    print(f"\nNext steps:")
    print(f"1. Generate mesh: gmsh -1 {output_file} -o {output_file[:-4]}.msh")
    print(f"2. Visualize: python3 scripts/visualize_nerve_tumor.py {tumor_file} {output_file}")
    print(f"3. Update config to use: {output_file[:-4]}.msh")

if __name__ == "__main__":
    main()