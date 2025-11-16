#!/usr/bin/env python3
"""
Simple Surface Nerve Generator
Creates a clean, smooth nerve rod that lies on tumor surface without zigzag patterns

This approach uses simple geometric placement instead of complex surface projection
"""

import sys
import os
import numpy as np

def parse_tumor_bbox(msh_file):
    """Extract just the bounding box from tumor mesh (simple and fast)"""
    vertices = []
    
    try:
        with open(msh_file, 'r') as f:
            lines = f.readlines()
        
        # Simple vertex extraction (first 1000 vertices for bbox estimation)
        for i, line in enumerate(lines):
            if line.strip() == '$Nodes':
                # Skip header lines and read some vertices
                start_line = i + 2
                for j in range(start_line, min(start_line + 1000, len(lines))):
                    parts = lines[j].strip().split()
                    if len(parts) >= 3:
                        try:
                            # Try different formats
                            if len(parts) >= 4 and parts[0].isdigit():
                                # Format: id x y z
                                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            else:
                                # Format: x y z
                                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            vertices.append([x, y, z])
                            if len(vertices) >= 500:  # Enough for good bbox estimate
                                break
                        except ValueError:
                            continue
                break
        
        if len(vertices) == 0:
            # Try Gmsh 4.x format
            reading_coords = False
            for line in lines:
                if reading_coords:
                    parts = line.strip().split()
                    if len(parts) >= 3 and line.strip() != '$EndNodes':
                        try:
                            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                            vertices.append([x, y, z])
                            if len(vertices) >= 500:
                                break
                        except ValueError:
                            continue
                    elif line.strip() == '$EndNodes':
                        break
                elif 'total nodes:' in line.lower() or line.strip().split()[0:2] == ['42722', 'nodes']:
                    reading_coords = True
        
        if len(vertices) == 0:
            print("Could not parse vertices, using default bbox")
            return np.array([[-30, 0, -10], [-10, 20, 10]])  # Default neuroma-like bbox
        
        vertices = np.array(vertices)
        bbox_min = np.min(vertices, axis=0)
        bbox_max = np.max(vertices, axis=0)
        
        print(f"Estimated tumor bbox from {len(vertices)} vertices:")
        print(f"  Min: [{bbox_min[0]:.1f}, {bbox_min[1]:.1f}, {bbox_min[2]:.1f}]")
        print(f"  Max: [{bbox_max[0]:.1f}, {bbox_max[1]:.1f}, {bbox_max[2]:.1f}]")
        
        return np.array([bbox_min, bbox_max])
        
    except Exception as e:
        print(f"Error parsing mesh: {e}")
        return np.array([[-30, 0, -10], [-10, 20, 10]])

def generate_smooth_surface_nerve(bbox, config_name):
    """Generate a smooth nerve that lies on tumor surface"""
    
    bbox_min, bbox_max = bbox[0], bbox[1]
    bbox_center = (bbox_min + bbox_max) / 2
    bbox_size = bbox_max - bbox_min
    
    print(f"Generating {config_name} nerve...")
    
    if config_name == "top_long_nerve":
        # Long nerve running across top surface (Y direction)
        surface_z = bbox_max[2] - 1.0  # 1cm below top surface
        start_y = bbox_min[1] + bbox_size[1] * 0.1  # Start 10% from back
        end_y = bbox_max[1] - bbox_size[1] * 0.1    # End 10% from front
        nerve_length = end_y - start_y
        
        # Generate smooth points along Y direction
        num_points = max(15, int(nerve_length / 1.5))  # ~1.5cm spacing
        points = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            y = start_y + t * nerve_length
            x = bbox_center[0]  # Center in X
            z = surface_z
            points.append([x, y, z])
        
        print(f"  Length: {nerve_length:.1f}cm, Points: {num_points}")
        
    elif config_name == "side_long_nerve":
        # Long nerve running along side surface (Z direction)
        surface_x = bbox_max[0] - 1.0  # 1cm from right side
        start_z = bbox_min[2] + bbox_size[2] * 0.15
        end_z = bbox_max[2] - bbox_size[2] * 0.15
        nerve_length = end_z - start_z
        
        num_points = max(12, int(nerve_length / 2.0))  # ~2cm spacing
        points = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            z = start_z + t * nerve_length
            x = surface_x
            y = bbox_center[1]  # Center in Y
            points.append([x, y, z])
            
    elif config_name == "front_long_nerve":
        # Long nerve running across front surface (X direction)
        surface_y = bbox_max[1] - 1.0  # 1cm from front
        start_x = bbox_min[0] + bbox_size[0] * 0.1
        end_x = bbox_max[0] - bbox_size[0] * 0.1
        nerve_length = end_x - start_x
        
        num_points = max(15, int(nerve_length / 2.0))
        points = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            x = start_x + t * nerve_length
            y = surface_y
            z = bbox_center[2]
            points.append([x, y, z])
            
    elif config_name == "diagonal_surface_nerve":
        # Diagonal nerve across top surface
        surface_z = bbox_max[2] - 0.8
        start_point = np.array([bbox_min[0] + bbox_size[0] * 0.2, bbox_min[1] + bbox_size[1] * 0.2, surface_z])
        end_point = np.array([bbox_max[0] - bbox_size[0] * 0.2, bbox_max[1] - bbox_size[1] * 0.2, surface_z])
        
        nerve_length = np.linalg.norm(end_point - start_point)
        num_points = max(20, int(nerve_length / 1.5))
        
        points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start_point + t * (end_point - start_point)
            points.append(point.tolist())
    
    else:
        # Default: simple top nerve
        surface_z = bbox_max[2] - 1.0
        start_y = bbox_center[1] - bbox_size[1] * 0.3
        end_y = bbox_center[1] + bbox_size[1] * 0.3
        
        points = []
        for i in range(15):
            t = i / 14
            y = start_y + t * (end_y - start_y)
            points.append([bbox_center[0], y, surface_z])
    
    return np.array(points)

def write_clean_nerve_geo(points, output_file, config_name):
    """Write nerve geometry file with proper spacing"""
    
    with open(output_file, 'w') as f:
        f.write(f"// Clean surface nerve: {config_name}\n")
        f.write(f"// Smooth nerve that follows tumor surface without zigzag\n")
        f.write(f"// Points: {len(points)}\n\n")
        
        # Write points with appropriate mesh size
        mesh_size = 2e-3  # 2mm mesh size for smooth curves
        for i, point in enumerate(points):
            f.write(f"Point({i+1}) = {{{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}, {mesh_size}}};\n")
        
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
        print("Usage: python3 generate_clean_surface_nerve.py <tumor.msh> [config]")
        print("\nAvailable configurations:")
        print("  top_long_nerve: Long nerve across tumor top surface (default)")
        print("  side_long_nerve: Long nerve along tumor side")
        print("  front_long_nerve: Long nerve across tumor front")
        print("  diagonal_surface_nerve: Diagonal nerve across surface")
        print(f"\nExample: python3 generate_clean_surface_nerve.py neuroma.msh top_long_nerve")
        sys.exit(1)
    
    tumor_file = sys.argv[1]
    config_name = sys.argv[2] if len(sys.argv) > 2 else 'top_long_nerve'
    output_file = f"clean_{config_name}.geo"
    
    if not os.path.exists(tumor_file):
        print(f"Error: File '{tumor_file}' not found")
        sys.exit(1)
    
    print(f"Creating clean surface nerve: {config_name}")
    
    # Get tumor bounding box
    bbox = parse_tumor_bbox(tumor_file)
    
    # Generate smooth nerve points
    nerve_points = generate_smooth_surface_nerve(bbox, config_name)
    
    if len(nerve_points) == 0:
        print("Error: Failed to generate nerve points")
        sys.exit(1)
    
    # Write geometry file
    write_clean_nerve_geo(nerve_points, output_file, config_name)
    
    print(f"\nGenerated clean nerve:")
    print(f"  Points: {len(nerve_points)}")
    print(f"  Output: {output_file}")
    
    # Print nerve extents
    nerve_min = np.min(nerve_points, axis=0)
    nerve_max = np.max(nerve_points, axis=0)
    nerve_length = np.linalg.norm(nerve_max - nerve_min)
    
    print(f"  Extent: [{nerve_min[0]:.1f}, {nerve_min[1]:.1f}, {nerve_min[2]:.1f}] to [{nerve_max[0]:.1f}, {nerve_max[1]:.1f}, {nerve_max[2]:.1f}]")
    print(f"  Length: {nerve_length:.1f}cm")
    
    print(f"\nNext steps:")
    print(f"1. Generate mesh: gmsh -1 {output_file} -o {output_file[:-4]}.msh")
    print(f"2. Visualize: python3 scripts/visualize_nerve_tumor.py {tumor_file} {output_file}")
    print(f"3. Update config to use: {output_file[:-4]}.msh")

if __name__ == "__main__":
    main()