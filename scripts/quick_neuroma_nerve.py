#!/usr/bin/env python3
"""
Simple Clean Nerve Generator for Neuroma
Uses known neuroma dimensions to create clean surface nerve
"""

import numpy as np

def generate_neuroma_surface_nerve():
    """Generate clean nerve using known neuroma dimensions"""
    
    # Known neuroma bounding box from previous analysis
    bbox_min = np.array([-33.4, 0.6, -14.1])
    bbox_max = np.array([-2.2, 24.7, 15.3])
    bbox_center = (bbox_min + bbox_max) / 2
    bbox_size = bbox_max - bbox_min
    
    print(f"Using known neuroma dimensions:")
    print(f"  Min: [{bbox_min[0]:.1f}, {bbox_min[1]:.1f}, {bbox_min[2]:.1f}]")
    print(f"  Max: [{bbox_max[0]:.1f}, {bbox_max[1]:.1f}, {bbox_max[2]:.1f}]")
    print(f"  Size: [{bbox_size[0]:.1f} × {bbox_size[1]:.1f} × {bbox_size[2]:.1f}] cm")
    
    # Generate long nerve on top surface, running from back to front (Y direction)
    surface_z = bbox_max[2] - 1.0  # 1cm below top surface for contact
    start_y = bbox_min[1] + 2.0    # Start 2cm from back
    end_y = bbox_max[1] - 2.0      # End 2cm from front
    nerve_x = bbox_center[0]       # Center in X direction
    
    nerve_length = end_y - start_y
    num_points = 20  # Good spacing for ~22cm length
    
    print(f"Generating nerve:")
    print(f"  Position: X={nerve_x:.1f}, Y=[{start_y:.1f} to {end_y:.1f}], Z={surface_z:.1f}")
    print(f"  Length: {nerve_length:.1f}cm, Points: {num_points}")
    
    # Generate smooth, evenly spaced points
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        y = start_y + t * nerve_length
        points.append([nerve_x, y, surface_z])
    
    return np.array(points)

def write_nerve_geo(points, filename):
    """Write nerve geometry file"""
    
    with open(filename, 'w') as f:
        f.write("// Clean neuroma surface nerve\n")
        f.write("// Long straight nerve on tumor surface for adhesion testing\n")
        f.write(f"// Points: {len(points)}, Length: ~20cm\n\n")
        
        # Write points with fine mesh size for smooth simulation
        mesh_size = 1e-3  # 1mm mesh size
        for i, point in enumerate(points):
            f.write(f"Point({i+1}) = {{{point[0]:.6f}, {point[1]:.6f}, {point[2]:.6f}, {mesh_size}}};\n")
        
        f.write("\n")
        
        # Write connecting lines
        for i in range(len(points) - 1):
            f.write(f"Line({i+1}) = {{{i+1}, {i+2}}};\n")
        
        f.write("\n")
        
        # Physical group for nerve constraints
        if len(points) > 1:
            line_list = ", ".join(str(i+1) for i in range(len(points) - 1))
            f.write(f'Physical Line("nerve_edge") = {{{line_list}}};\n')

def main():
    print("Generating clean neuroma surface nerve...")
    
    # Generate nerve points
    nerve_points = generate_neuroma_surface_nerve()
    
    # Write geometry file
    output_file = "neuroma_surface_nerve.geo"
    write_nerve_geo(nerve_points, output_file)
    
    print(f"\nSuccess! Created: {output_file}")
    print(f"Nerve statistics:")
    
    nerve_min = np.min(nerve_points, axis=0)
    nerve_max = np.max(nerve_points, axis=0)
    nerve_length = np.linalg.norm(nerve_max - nerve_min)
    
    print(f"  Start: [{nerve_min[0]:.1f}, {nerve_min[1]:.1f}, {nerve_min[2]:.1f}]")
    print(f"  End:   [{nerve_max[0]:.1f}, {nerve_max[1]:.1f}, {nerve_max[2]:.1f}]")
    print(f"  Length: {nerve_length:.1f}cm")
    
    print(f"\nNext steps:")
    print(f"1. gmsh -1 {output_file} -o {output_file[:-4]}.msh")
    print(f"2. python3 scripts/visualize_nerve_tumor.py resource/tissue/neuroma_refined_uniform.msh {output_file}")

if __name__ == "__main__":
    main()