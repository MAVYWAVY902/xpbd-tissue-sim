#!/usr/bin/env python3
"""
Fix mesh coordinates: rescale and recenter meshes to reasonable coordinate range.
This preserves relative alignment between meshes while bringing them to origin.

Usage:
    python3 fix_mesh_coordinates.py --input1 tumor.msh --input2 bone.obj --scale 0.001 --output-dir fixed/
    
The script will:
1. Load both meshes
2. Compute their combined bounding box
3. Apply scaling factor (e.g., 0.001 to convert mm to m)
4. Center them together at origin (preserving relative alignment)
5. Save fixed meshes
"""

import argparse
import sys
import os

def read_obj(filename):
    """Read OBJ file and return vertices and faces"""
    vertices = []
    faces = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('v '):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith('f '):
                parts = line.split()
                # Handle both "f v1 v2 v3" and "f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3" formats
                face = []
                for p in parts[1:]:
                    vertex_idx = int(p.split('/')[0])
                    face.append(vertex_idx)
                faces.append(face)
    
    return vertices, faces

def write_obj(filename, vertices, faces):
    """Write OBJ file"""
    with open(filename, 'w') as f:
        f.write("# Fixed mesh coordinates\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in faces:
            f.write(f"f {' '.join(map(str, face))}\n")

def read_gmsh(filename):
    """Read GMSH .msh file (version 4.1) and return entities, vertices and element blocks"""
    entities_section = []
    vertices = []
    element_blocks = []  # List of (entityDim, entityTag, elementType, elements)
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        
        # Look for $Entities section
        if line == '$Entities':
            i += 1
            # Read until $EndEntities
            while i < len(lines) and lines[i].strip() != '$EndEntities':
                entities_section.append(lines[i])
                i += 1
            i += 1  # Skip $EndEntities
            continue
        
        # Look for $Nodes section
        if line == '$Nodes':
            i += 1
            # Read header: numEntityBlocks numNodes minNodeTag maxNodeTag
            header = lines[i].strip().split()
            num_entity_blocks = int(header[0])
            num_nodes = int(header[1])
            print(f"Reading {num_nodes} nodes from {num_entity_blocks} entity blocks...")
            
            i += 1
            for block_idx in range(num_entity_blocks):
                # Entity block header: entityDim entityTag parametric numNodesInBlock
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
                    vertices.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    i += 1
            
            # Skip to $EndNodes
            while i < len(lines) and lines[i].strip() != '$EndNodes':
                i += 1
        
        # Look for $Elements section
        elif line == '$Elements':
            i += 1
            header = lines[i].strip().split()
            num_entity_blocks = int(header[0])
            num_elements = int(header[1])
            print(f"Reading {num_elements} elements from {num_entity_blocks} entity blocks...")
            
            i += 1
            for block_idx in range(num_entity_blocks):
                # Entity block header: entityDim entityTag elementType numElementsInBlock
                block_header = lines[i].strip().split()
                entity_dim = int(block_header[0])
                entity_tag = int(block_header[1])
                element_type = int(block_header[2])
                num_elements_in_block = int(block_header[3])
                i += 1
                
                # Read elements
                block_elements = []
                for _ in range(num_elements_in_block):
                    parts = lines[i].strip().split()
                    element_tag = int(parts[0])
                    node_indices = [int(x) for x in parts[1:]]
                    block_elements.append(node_indices)
                    i += 1
                
                element_blocks.append((entity_dim, entity_tag, element_type, block_elements))
            
            # Skip to $EndElements
            while i < len(lines) and lines[i].strip() != '$EndElements':
                i += 1
        
        i += 1
    
    return entities_section, vertices, element_blocks
def write_gmsh(filename, entities_section, vertices, element_blocks):
    """Write GMSH .msh file (version 4.1) with proper $Entities section"""
    with open(filename, 'w') as f:
        f.write("$MeshFormat\n")
        f.write("4.1 0 8\n")
        f.write("$EndMeshFormat\n")
        
        # Write preserved entities section
        f.write("$Entities\n")
        for line in entities_section:
            f.write(line)
        f.write("$EndEntities\n")
        
        # Group nodes by (entity_dim, entity_tag), avoiding duplicates
        # Nodes should appear in only ONE entity block. Priority: higher dimension wins
        # (e.g., boundary nodes belong to surface entity, not volume)
        node_to_entity = {}  # node_id -> (entity_dim, entity_tag)
        for entity_dim, entity_tag, element_type, block_elements in element_blocks:
            for nodes in block_elements:
                for node in nodes:
                    # Assign node to current entity if not yet assigned, 
                    # or if current entity has LOWER dimension (surface nodes belong to surface, not volume)
                    if node not in node_to_entity or entity_dim < node_to_entity[node][0]:
                        node_to_entity[node] = (entity_dim, entity_tag)
        
        # Now group nodes by entity
        nodes_by_entity = {}
        for node, (entity_dim, entity_tag) in node_to_entity.items():
            key = (entity_dim, entity_tag)
            if key not in nodes_by_entity:
                nodes_by_entity[key] = set()
            nodes_by_entity[key].add(node)
        
        # Write nodes - for each entity, write node tags then coordinates
        f.write("$Nodes\n")
        total_nodes = len(vertices)
        f.write(f"{len(nodes_by_entity)} {total_nodes} 1 {total_nodes}\n")
        
        for (entity_dim, entity_tag), node_set in sorted(nodes_by_entity.items()):
            num_nodes = len(node_set)
            f.write(f"{entity_dim} {entity_tag} 0 {num_nodes}\n")
            
            # Write node tags for this entity
            sorted_nodes = sorted(node_set)
            for node in sorted_nodes:
                f.write(f"{node}\n")
            
            # Write coordinates for this entity's nodes
            for node in sorted_nodes:
                v = vertices[node - 1]  # node IDs are 1-based
                f.write(f"{v[0]:.10f} {v[1]:.10f} {v[2]:.10f}\n")
        
        f.write("$EndNodes\n")
        
        # Group elements by (entity_dim, entity_tag, element_type)
        elements_by_entity_type = {}
        for entity_dim, entity_tag, element_type, block_elements in element_blocks:
            key = (entity_dim, entity_tag, element_type)
            if key not in elements_by_entity_type:
                elements_by_entity_type[key] = []
            elements_by_entity_type[key].extend(block_elements)
        
        # Write elements
        total_elements = sum(len(elems) for elems in elements_by_entity_type.values())
        f.write("$Elements\n")
        f.write(f"{len(elements_by_entity_type)} {total_elements} 1 {total_elements}\n")
        
        element_idx = 1
        for (entity_dim, entity_tag, element_type), block_elements in sorted(elements_by_entity_type.items()):
            f.write(f"{entity_dim} {entity_tag} {element_type} {len(block_elements)}\n")
            for nodes in block_elements:
                f.write(f"{element_idx} {' '.join(map(str, nodes))}\n")
                element_idx += 1
        
        f.write("$EndElements\n")

def compute_bbox(vertices):
    """Compute bounding box [min_x, min_y, min_z, max_x, max_y, max_z]"""
    if not vertices:
        return [0, 0, 0, 0, 0, 0]
    
    min_x = min(v[0] for v in vertices)
    min_y = min(v[1] for v in vertices)
    min_z = min(v[2] for v in vertices)
    max_x = max(v[0] for v in vertices)
    max_y = max(v[1] for v in vertices)
    max_z = max(v[2] for v in vertices)
    
    return [min_x, min_y, min_z, max_x, max_y, max_z]

def main():
    parser = argparse.ArgumentParser(description='Fix mesh coordinates: rescale and recenter')
    parser.add_argument('--input1', required=True, help='First input mesh file (.obj or .msh)')
    parser.add_argument('--input2', required=True, help='Second input mesh file (.obj or .msh)')
    parser.add_argument('--scale', type=float, default=0.001, help='Scaling factor (default: 0.001 to convert mm to m)')
    parser.add_argument('--output-dir', required=True, help='Output directory for fixed meshes')
    
    args = parser.parse_args()
    
    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    # Read meshes
    print(f"\nReading {args.input1}...")
    if args.input1.endswith('.obj'):
        verts1, data1 = read_obj(args.input1)
        entities1 = []
        is_obj1 = True
    else:
        entities1, verts1, data1 = read_gmsh(args.input1)
        is_obj1 = False
    
    print(f"Reading {args.input2}...")
    if args.input2.endswith('.obj'):
        verts2, data2 = read_obj(args.input2)
        entities2 = []
        is_obj2 = True
    else:
        entities2, verts2, data2 = read_gmsh(args.input2)
        is_obj2 = False
    
    # Compute combined bounding box
    all_verts = verts1 + verts2
    bbox = compute_bbox(all_verts)
    
    print(f"\nOriginal coordinate range:")
    print(f"  X: [{bbox[0]:.3f}, {bbox[3]:.3f}]")
    print(f"  Y: [{bbox[1]:.3f}, {bbox[4]:.3f}]")
    print(f"  Z: [{bbox[2]:.3f}, {bbox[5]:.3f}]")
    
    # Compute center of combined bounding box
    center = [(bbox[0] + bbox[3]) / 2,
              (bbox[1] + bbox[4]) / 2,
              (bbox[2] + bbox[5]) / 2]
    
    print(f"\nCenter: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})")
    print(f"Applying scale factor: {args.scale}")
    
    # Transform vertices: scale and center
    def transform_vertices(verts):
        new_verts = []
        for v in verts:
            new_v = [
                (v[0] - center[0]) * args.scale,
                (v[1] - center[1]) * args.scale,
                (v[2] - center[2]) * args.scale
            ]
            new_verts.append(new_v)
        return new_verts
    
    verts1_fixed = transform_vertices(verts1)
    verts2_fixed = transform_vertices(verts2)
    
    # Compute new bounding box
    all_verts_fixed = verts1_fixed + verts2_fixed
    bbox_fixed = compute_bbox(all_verts_fixed)
    
    print(f"\nFixed coordinate range:")
    print(f"  X: [{bbox_fixed[0]:.6f}, {bbox_fixed[3]:.6f}]")
    print(f"  Y: [{bbox_fixed[1]:.6f}, {bbox_fixed[4]:.6f}]")
    print(f"  Z: [{bbox_fixed[2]:.6f}, {bbox_fixed[5]:.6f}]")
    
    # Write output files
    basename1 = os.path.basename(args.input1)
    basename2 = os.path.basename(args.input2)
    name1, ext1 = os.path.splitext(basename1)
    name2, ext2 = os.path.splitext(basename2)
    
    output1 = os.path.join(args.output_dir, f"{name1}_fixed{ext1}")
    output2 = os.path.join(args.output_dir, f"{name2}_fixed{ext2}")
    
    print(f"\nWriting {output1}...")
    if is_obj1:
        write_obj(output1, verts1_fixed, data1)
    else:
        write_gmsh(output1, entities1, verts1_fixed, data1)
    
    print(f"Writing {output2}...")
    if is_obj2:
        write_obj(output2, verts2_fixed, data2)
    else:
        write_gmsh(output2, entities2, verts2_fixed, data2)
    
    print("\n✅ Done! Now you can use these fixed meshes with use-original-coords: true")

if __name__ == '__main__':
    main()
