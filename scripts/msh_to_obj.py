#!/usr/bin/env python3
"""
Convert GMSH MSH file to OBJ (surface only)
Extracts surface triangles from tetrahedral mesh
"""

import sys
import struct

def read_msh_file(filename):
    """Read GMSH MSH file and extract surface triangles"""
    vertices = []
    triangles = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Read nodes
        if line == '$Nodes':
            i += 1
            header = lines[i].strip().split()
            
            # MSH format 4: numEntityBlocks numNodes minNodeTag maxNodeTag
            if len(header) == 4:
                num_entity_blocks = int(header[0])
                total_nodes = int(header[1])
                vertices = [None] * total_nodes  # Pre-allocate
                i += 1
                
                for _ in range(num_entity_blocks):
                    # Entity header: entityDim entityTag parametric numNodes
                    entity_header = lines[i].strip().split()
                    num_nodes_in_block = int(entity_header[3])
                    i += 1
                    
                    # Read node tags
                    node_tags = []
                    for _ in range(num_nodes_in_block):
                        node_tags.append(int(lines[i].strip()) - 1)  # Convert to 0-indexed
                        i += 1
                    
                    # Read node coordinates
                    for j in range(num_nodes_in_block):
                        parts = lines[i].strip().split()
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        vertices[node_tags[j]] = (x, y, z)
                        i += 1
            else:
                # Old format: just num_nodes
                num_nodes = int(header[0])
                i += 1
                for _ in range(num_nodes):
                    parts = lines[i].strip().split()
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    vertices.append((x, y, z))
                    i += 1
            i += 1  # Skip $EndNodes
        
        # Read elements
        elif line == '$Elements':
            i += 1
            header = lines[i].strip().split()
            
            # MSH format 4: numEntityBlocks numElements minElementTag maxElementTag
            if len(header) == 4:
                num_entity_blocks = int(header[0])
                i += 1
                
                for _ in range(num_entity_blocks):
                    # Entity header: entityDim entityTag elementType numElements
                    entity_header = lines[i].strip().split()
                    elem_type = int(entity_header[2])
                    num_elements_in_block = int(entity_header[3])
                    i += 1
                    
                    # Element type 2 = triangle (surface face)
                    if elem_type == 2:
                        for _ in range(num_elements_in_block):
                            parts = lines[i].strip().split()
                            # Format: elementTag vertex1 vertex2 vertex3
                            v1 = int(parts[1]) - 1
                            v2 = int(parts[2]) - 1
                            v3 = int(parts[3]) - 1
                            triangles.append((v1, v2, v3))
                            i += 1
                    else:
                        # Skip non-triangle elements
                        for _ in range(num_elements_in_block):
                            i += 1
            else:
                # Old format
                num_elements = int(header[0])
                i += 1
                for _ in range(num_elements):
                    parts = lines[i].strip().split()
                    elem_type = int(parts[1])
                    
                    if elem_type == 2:
                        v1 = int(parts[-3]) - 1
                        v2 = int(parts[-2]) - 1
                        v3 = int(parts[-1]) - 1
                        triangles.append((v1, v2, v3))
                    i += 1
            i += 1  # Skip $EndElements
        
        else:
            i += 1
    
    return vertices, triangles

def write_obj_file(filename, vertices, triangles):
    """Write OBJ file"""
    with open(filename, 'w') as f:
        f.write(f"# Converted from MSH\n")
        f.write(f"# Vertices: {len(vertices)}\n")
        f.write(f"# Faces: {len(triangles)}\n\n")
        
        # Write vertices
        for v in vertices:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
        
        f.write("\n")
        
        # Write faces (OBJ uses 1-based indexing)
        for t in triangles:
            f.write(f"f {t[0]+1} {t[1]+1} {t[2]+1}\n")

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 msh_to_obj.py <input.msh> <output.obj>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    print(f"Reading {input_file}...")
    vertices, triangles = read_msh_file(input_file)
    
    print(f"  Vertices: {len(vertices)}")
    print(f"  Triangles: {len(triangles)}")
    
    if len(triangles) == 0:
        print("WARNING: No surface triangles found!")
    
    print(f"Writing {output_file}...")
    write_obj_file(output_file, vertices, triangles)
    
    print("Done!")

if __name__ == '__main__':
    main()
