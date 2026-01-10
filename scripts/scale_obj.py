#!/usr/bin/env python3
"""
Script to scale OBJ files by a given factor.
Preserves all file structure including comments, materials, and other directives.
Only vertex coordinates are scaled.

Usage:
    python scale_obj.py input.obj output.obj scale_factor
    python scale_obj.py input.obj output.obj 0.001
"""

import sys
import os


def scale_obj_file(input_path, output_path, scale_factor):
    """
    Scale an OBJ file by the given factor.
    
    Args:
        input_path: Path to input OBJ file
        output_path: Path to output OBJ file
        scale_factor: Scaling factor to apply to vertices
    """
    print(f"Reading: {input_path}")
    print(f"Scale factor: {scale_factor}")
    
    vertices_scaled = 0
    lines_processed = 0
    
    with open(input_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            lines_processed += 1
            
            # Show progress for large files
            if lines_processed % 100000 == 0:
                print(f"Processed {lines_processed} lines, scaled {vertices_scaled} vertices...")
            
            # Only modify vertex lines (starting with "v ")
            if line.startswith('v '):
                parts = line.split()
                if len(parts) >= 4:
                    # Scale x, y, z coordinates
                    x = float(parts[1]) * scale_factor
                    y = float(parts[2]) * scale_factor
                    z = float(parts[3]) * scale_factor
                    
                    # Preserve any additional vertex data (e.g., color)
                    remaining = ' '.join(parts[4:])
                    if remaining:
                        outfile.write(f"v {x} {y} {z} {remaining}\n")
                    else:
                        outfile.write(f"v {x} {y} {z}\n")
                    
                    vertices_scaled += 1
                else:
                    # Malformed vertex line, keep as is
                    outfile.write(line)
            else:
                # Keep all other lines unchanged (vt, vn, f, mtllib, usemtl, comments, etc.)
                outfile.write(line)
    
    print(f"Done! Processed {lines_processed} lines, scaled {vertices_scaled} vertices.")
    print(f"Output saved to: {output_path}")


def main():
    if len(sys.argv) < 3:
        print("Usage: python scale_obj.py <input.obj> <output.obj> [scale_factor]")
        print("Example: python scale_obj.py tbone_origin.obj tbone_scaled.obj 0.001")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    scale_factor = float(sys.argv[3]) if len(sys.argv) > 3 else 0.001
    
    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found!")
        sys.exit(1)
    
    # Confirm overwrite if output exists
    if os.path.exists(output_file):
        response = input(f"Warning: '{output_file}' already exists. Overwrite? (y/n): ")
        if response.lower() != 'y':
            print("Cancelled.")
            sys.exit(0)
    
    scale_obj_file(input_file, output_file, scale_factor)


if __name__ == "__main__":
    main()
