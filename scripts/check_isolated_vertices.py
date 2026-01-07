#!/usr/bin/env python3
"""
Check for isolated vertices (vertices not belonging to any tetrahedron)
"""

def read_msh_file(filename):
    """Parse GMSH .msh file to get vertices and elements"""
    vertices = []
    tetrahedra = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # Detect format version
    version = 2
    for line in lines:
        if line.strip().startswith('$MeshFormat'):
            idx = lines.index(line)
            version_line = lines[idx + 1].strip().split()[0]
            version = int(float(version_line))
            print(f"GMSH format version: {version_line}")
            break
    
    # Find $Nodes section
    i = 0
    while i < len(lines):
        line = lines[i]
        
        if line.strip() == '$Nodes':
            i += 1
            
            if version >= 4:
                # Version 4.x format
                header = lines[i].strip().split()
                num_entity_blocks = int(header[0])
                num_nodes = int(header[1])
                print(f"Number of nodes: {num_nodes} in {num_entity_blocks} entity blocks")
                i += 1
                
                for block_idx in range(num_entity_blocks):
                    block_header = lines[i].strip().split()
                    num_nodes_in_block = int(block_header[3])
                    i += 1
                    
                    # Skip node tags
                    for _ in range(num_nodes_in_block):
                        i += 1
                    
                    # Read coordinates
                    for _ in range(num_nodes_in_block):
                        parts = lines[i].strip().split()
                        vertex = [float(parts[0]), float(parts[1]), float(parts[2])]
                        vertices.append(vertex)
                        i += 1
            else:
                # Version 2.x format
                num_nodes = int(lines[i].strip())
                print(f"Number of nodes: {num_nodes}")
                i += 1
                
                for _ in range(num_nodes):
                    parts = lines[i].strip().split()
                    if len(parts) >= 4:
                        vertex = [float(parts[1]), float(parts[2]), float(parts[3])]
                        vertices.append(vertex)
                    i += 1
            break
        i += 1
    
    # Find $Elements section
    i = 0
    while i < len(lines):
        line = lines[i]
        
        if line.strip() == '$Elements':
            i += 1
            
            if version >= 4:
                # Version 4.x format
                header = lines[i].strip().split()
                num_entity_blocks = int(header[0])
                num_elements = int(header[1])
                print(f"Number of elements: {num_elements} in {num_entity_blocks} entity blocks")
                i += 1
                
                for block_idx in range(num_entity_blocks):
                    block_header = lines[i].strip().split()
                    element_type = int(block_header[2])
                    num_elements_in_block = int(block_header[3])
                    i += 1
                    
                    for _ in range(num_elements_in_block):
                        parts = lines[i].strip().split()
                        # Element type 4 = tetrahedron (4 nodes)
                        if element_type == 4:
                            # Format: elementTag node1 node2 node3 node4
                            # Nodes are 1-indexed in GMSH, convert to 0-indexed
                            tet = [int(parts[1])-1, int(parts[2])-1, int(parts[3])-1, int(parts[4])-1]
                            tetrahedra.append(tet)
                        i += 1
            else:
                # Version 2.x format
                num_elements = int(lines[i].strip())
                print(f"Number of elements: {num_elements}")
                i += 1
                
                for _ in range(num_elements):
                    parts = lines[i].strip().split()
                    element_type = int(parts[1])
                    # Element type 4 = tetrahedron
                    if element_type == 4:
                        num_tags = int(parts[2])
                        # nodes start after: elementId, elementType, numTags, tags...
                        node_start = 3 + num_tags
                        tet = [int(parts[node_start])-1, int(parts[node_start+1])-1, 
                               int(parts[node_start+2])-1, int(parts[node_start+3])-1]
                        tetrahedra.append(tet)
                    i += 1
            break
        i += 1
    
    return vertices, tetrahedra


if __name__ == "__main__":
    mesh_file = "resource/tissue/neuroma_tet.msh"
    
    print(f"Reading mesh file: {mesh_file}\n")
    vertices, tetrahedra = read_msh_file(mesh_file)
    
    print(f"\nTotal vertices: {len(vertices)}")
    print(f"Total tetrahedra: {len(tetrahedra)}")
    
    # Find which vertices are used in tetrahedra
    used_vertices = set()
    for tet in tetrahedra:
        for v in tet:
            used_vertices.add(v)
    
    # Find isolated vertices
    isolated = []
    for i in range(len(vertices)):
        if i not in used_vertices:
            isolated.append(i)
    
    print(f"\n{'='*60}")
    print(f"Isolated vertices (not in any tetrahedron): {len(isolated)}")
    print(f"{'='*60}")
    
    if isolated:
        print(f"\nThese vertices will have ZERO mass and act as FIXED points!")
        for i in isolated[:20]:  # Show first 20
            v = vertices[i]
            print(f"  Vertex {i}: [{v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f}]")
        
        if len(isolated) > 20:
            print(f"  ... and {len(isolated) - 20} more")
        
        # Compute center after transformation
        center = [0.0, 0.0, 0.0]
        for v in vertices:
            center[0] += v[0]
            center[1] += v[1]
            center[2] += v[2]
        center[0] /= len(vertices)
        center[1] /= len(vertices)
        center[2] /= len(vertices)
        
        print(f"\nAfter moving to position [0, 0, 0], these isolated vertices will be at:")
        for i in isolated[:10]:
            v = vertices[i]
            transformed = [v[0] - center[0], v[1] - center[1], v[2] - center[2]]
            dist = (transformed[0]**2 + transformed[1]**2 + transformed[2]**2)**0.5
            print(f"  Vertex {i}: [{transformed[0]:.6f}, {transformed[1]:.6f}, {transformed[2]:.6f}] (dist: {dist:.6f})")
    else:
        print("\nNo isolated vertices found - all vertices belong to at least one tetrahedron")
    
    # Compute statistics
    vertex_valence = [0] * len(vertices)
    for tet in tetrahedra:
        for v in tet:
            vertex_valence[v] += 1
    
    print(f"\n{'='*60}")
    print(f"Vertex valence statistics:")
    print(f"{'='*60}")
    print(f"  Min valence: {min(vertex_valence)}")
    print(f"  Max valence: {max(vertex_valence)}")
    print(f"  Average valence: {sum(vertex_valence) / len(vertex_valence):.2f}")
