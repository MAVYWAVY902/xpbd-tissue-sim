#!/usr/bin/env python3
"""
Check vertices in the tumor mesh to see if any are at or near the origin
"""
import sys

# Try to read .msh file manually without numpy
def read_msh_file(filename):
    """Simple parser for GMSH .msh format (version 2.2 and 4.x)"""
    vertices = []
    
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
    in_nodes = False
    i = 0
    while i < len(lines):
        line = lines[i]
        
        if line.strip() == '$Nodes':
            in_nodes = True
            i += 1
            
            # Parse header based on version
            if version >= 4:
                # Version 4.x format: numEntityBlocks numNodes minNodeTag maxNodeTag
                header = lines[i].strip().split()
                num_entity_blocks = int(header[0])
                num_nodes = int(header[1])
                print(f"Number of nodes: {num_nodes} in {num_entity_blocks} entity blocks")
                i += 1
                
                # Read entity blocks
                for block_idx in range(num_entity_blocks):
                    # Block header: entityDim entityTag parametric numNodesInBlock
                    block_header = lines[i].strip().split()
                    num_nodes_in_block = int(block_header[3])
                    i += 1
                    
                    # Read node tags
                    node_tags = []
                    for _ in range(num_nodes_in_block):
                        node_tags.append(int(lines[i].strip()))
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
                    if len(parts) >= 4:  # node_id x y z
                        vertex = [float(parts[1]), float(parts[2]), float(parts[3])]
                        vertices.append(vertex)
                    i += 1
            break
        
        i += 1
    
    return vertices

if __name__ == "__main__":
    mesh_file = "resource/tissue/neuroma_tet.msh"
    
    print(f"Reading mesh file: {mesh_file}")
    vertices = read_msh_file(mesh_file)
    
    print(f"\nTotal vertices: {len(vertices)}")
    
    if len(vertices) > 0:
        # Compute center of mass (assuming uniform density) - manual calculation
        center = [0.0, 0.0, 0.0]
        for v in vertices:
            center[0] += v[0]
            center[1] += v[1]
            center[2] += v[2]
        center[0] /= len(vertices)
        center[1] /= len(vertices)
        center[2] /= len(vertices)
        print(f"Geometric center: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]")
        
        # Helper function to compute distance
        def distance(v1, v2=[0, 0, 0]):
            dx = v1[0] - v2[0]
            dy = v1[1] - v2[1]
            dz = v1[2] - v2[2]
            return (dx*dx + dy*dy + dz*dz) ** 0.5
        
        # Find vertices near origin
        print("\n" + "="*60)
        print("Vertices at or near origin (distance < 0.1):")
        print("="*60)
        found_any = False
        for i, v in enumerate(vertices):
            dist = distance(v)
            if dist < 0.1:
                print(f"  Vertex {i}: [{v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f}] (distance: {dist:.6f})")
                found_any = True
        
        if not found_any:
            print("  None found")
        
        # Find vertices near geometric center
        print("\n" + "="*60)
        print(f"Vertices near geometric center (distance < 0.05):")
        print("="*60)
        found_any = False
        for i, v in enumerate(vertices):
            dist = distance(v, center)
            if dist < 0.05:
                print(f"  Vertex {i}: [{v[0]:.6f}, {v[1]:.6f}, {v[2]:.6f}] (distance from center: {dist:.6f})")
                found_any = True
        
        if not found_any:
            print("  None found")
        
        # Compute bounding box
        print("\n" + "="*60)
        print("Bounding box:")
        print("="*60)
        min_coords = [vertices[0][0], vertices[0][1], vertices[0][2]]
        max_coords = [vertices[0][0], vertices[0][1], vertices[0][2]]
        for v in vertices:
            min_coords[0] = min(min_coords[0], v[0])
            min_coords[1] = min(min_coords[1], v[1])
            min_coords[2] = min(min_coords[2], v[2])
            max_coords[0] = max(max_coords[0], v[0])
            max_coords[1] = max(max_coords[1], v[1])
            max_coords[2] = max(max_coords[2], v[2])
        
        size = [max_coords[0] - min_coords[0], 
                max_coords[1] - min_coords[1], 
                max_coords[2] - min_coords[2]]
        
        print(f"  Min: [{min_coords[0]:.6f}, {min_coords[1]:.6f}, {min_coords[2]:.6f}]")
        print(f"  Max: [{max_coords[0]:.6f}, {max_coords[1]:.6f}, {max_coords[2]:.6f}]")
        print(f"  Size: [{size[0]:.6f}, {size[1]:.6f}, {size[2]:.6f}]")
        
        # After transformation with position=[0,0,0], center will be at origin
        print("\n" + "="*60)
        print("After moving to position [0, 0, 0]:")
        print("="*60)
        transformed = []
        for v in vertices:
            transformed.append([v[0] - center[0], v[1] - center[1], v[2] - center[2]])
        
        new_center = [0.0, 0.0, 0.0]
        for v in transformed:
            new_center[0] += v[0]
            new_center[1] += v[1]
            new_center[2] += v[2]
        new_center[0] /= len(transformed)
        new_center[1] /= len(transformed)
        new_center[2] /= len(transformed)
        
        print(f"New geometric center: [{new_center[0]:.10f}, {new_center[1]:.10f}, {new_center[2]:.10f}]")
        
        print("\nVertices that will be at/near origin after transformation:")
        count = 0
        for i, v_transformed in enumerate(transformed):
            dist = distance(v_transformed)
            if dist < 0.1:
                print(f"  Vertex {i}: original [{vertices[i][0]:.6f}, {vertices[i][1]:.6f}, {vertices[i][2]:.6f}] -> [{v_transformed[0]:.6f}, {v_transformed[1]:.6f}, {v_transformed[2]:.6f}] (distance: {dist:.6f})")
                count += 1
        
        if count == 0:
            print("  None found within 0.1 units")
        else:
            print(f"\nTotal: {count} vertices near origin after transformation")

