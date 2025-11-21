#!/usr/bin/env python3
"""
Inspect tumor mesh to understand its triangle structure
"""

import sys

def read_msh_file(filename):
    """Read a Gmsh .msh file and extract mesh information"""
    
    vertices = []
    triangles = []
    tetrahedra = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Read nodes/vertices
        if line == '$Nodes':
            i += 1
            # Check for version 2.2 vs 4.1 format
            header = lines[i].strip().split()
            if len(header) == 1:  # Version 2.2
                num_nodes = int(header[0])
                i += 1
                for _ in range(num_nodes):
                    parts = lines[i].strip().split()
                    # Format: node_id x y z
                    vertex = [float(parts[1]), float(parts[2]), float(parts[3])]
                    vertices.append(vertex)
                    i += 1
            else:  # Version 4.1
                # Skip blocks format for now
                pass
                
        # Read elements
        elif line == '$Elements':
            i += 1
            header = lines[i].strip().split()
            if len(header) == 1:  # Version 2.2
                num_elements = int(header[0])
                i += 1
                for _ in range(num_elements):
                    parts = lines[i].strip().split()
                    elem_type = int(parts[1])
                    
                    # Element type 2 = 3-node triangle
                    # Element type 4 = 4-node tetrahedron
                    if elem_type == 2:
                        # Format: elem_id type num_tags [tags...] node1 node2 node3
                        num_tags = int(parts[2])
                        node_start_idx = 3 + num_tags
                        tri = [int(parts[node_start_idx]) - 1,     # Convert to 0-indexed
                               int(parts[node_start_idx + 1]) - 1,
                               int(parts[node_start_idx + 2]) - 1]
                        triangles.append(tri)
                    elif elem_type == 4:
                        # Tetrahedron
                        num_tags = int(parts[2])
                        node_start_idx = 3 + num_tags
                        tet = [int(parts[node_start_idx]) - 1,     # Convert to 0-indexed
                               int(parts[node_start_idx + 1]) - 1,
                               int(parts[node_start_idx + 2]) - 1,
                               int(parts[node_start_idx + 3]) - 1]
                        tetrahedra.append(tet)
                    i += 1
        else:
            i += 1
    
    return vertices, triangles, tetrahedra


def analyze_mesh(filename):
    """Analyze the mesh and print statistics"""
    
    print(f"Analyzing mesh file: {filename}")
    print("=" * 60)
    
    vertices, triangles, tetrahedra = read_msh_file(filename)
    
    print(f"\nMesh Statistics:")
    print(f"  Total vertices: {len(vertices)}")
    print(f"  Total triangles (surface faces): {len(triangles)}")
    print(f"  Total tetrahedra (volume elements): {len(tetrahedra)}")
    
    if len(vertices) > 0:
        xs = [v[0] for v in vertices]
        ys = [v[1] for v in vertices]
        zs = [v[2] for v in vertices]
        
        print(f"\nVertex bounds:")
        print(f"  X: [{min(xs):.4f}, {max(xs):.4f}]")
        print(f"  Y: [{min(ys):.4f}, {max(ys):.4f}]")
        print(f"  Z: [{min(zs):.4f}, {max(zs):.4f}]")
        
        # Calculate mesh size
        size_x = max(xs) - min(xs)
        size_y = max(ys) - min(ys)
        size_z = max(zs) - min(zs)
        print(f"  Mesh dimensions: {size_x:.4f} x {size_y:.4f} x {size_z:.4f}")
    
    if len(triangles) > 0:
        print(f"\nSample triangles (first 10):")
        for i, tri in enumerate(triangles[:10]):
            print(f"  Triangle {i}: vertices {tri}")
            if len(vertices) > 0 and all(idx < len(vertices) for idx in tri):
                v1, v2, v3 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
                centroid = [(v1[j] + v2[j] + v3[j]) / 3 for j in range(3)]
                print(f"    Centroid: ({centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f})")
        
        # Find which vertices are used in triangles
        if len(vertices) > 0:
            triangle_vertices = set()
            for tri in triangles:
                triangle_vertices.update(tri)
            print(f"\n  Surface vertices (in triangles): {len(triangle_vertices)} / {len(vertices)}")
    
    if len(tetrahedra) > 0:
        print(f"\nSample tetrahedra (first 5):")
        for i, tet in enumerate(tetrahedra[:5]):
            print(f"  Tetrahedron {i}: vertices {tet}")


if __name__ == "__main__":
    # Analyze the ellipsoid tumor mesh
    mesh_file = "/home/yunxin/xpbd-tissue-sim/resource/cube/cube8.msh"
    analyze_mesh(mesh_file)
