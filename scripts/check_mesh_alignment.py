#!/usr/bin/env python3
"""
Check mesh alignment after transformations
"""
import numpy as np
import sys

def load_obj(filename):
    vertices = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(vertices)

def load_msh_surface(filename):
    """Load surface vertices from GMSH MSH format 4"""
    vertices = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    i = 0
    while i < len(lines):
        if lines[i].strip() == '$Entities':
            i += 1
            break
        i += 1
    
    # Skip entity info
    while i < len(lines):
        if lines[i].strip() == '$EndEntities':
            i += 1
            break
        i += 1
    
    # Find Nodes section
    while i < len(lines):
        if lines[i].strip() == '$Nodes':
            i += 1
            break
        i += 1
    
    # Read nodes
    parts = lines[i].strip().split()
    num_entity_blocks = int(parts[0])
    total_nodes = int(parts[1])
    i += 1
    
    node_coords = {}
    
    for block in range(num_entity_blocks):
        entity_info = lines[i].strip().split()
        num_nodes_in_block = int(entity_info[3])
        i += 1
        
        # Read node tags
        node_tags = []
        for _ in range(num_nodes_in_block):
            node_tags.append(int(lines[i].strip()))
            i += 1
        
        # Read coordinates
        for tag in node_tags:
            coords = list(map(float, lines[i].strip().split()))
            node_coords[tag] = coords
            vertices.append(coords)
            i += 1
    
    return np.array(vertices)

def analyze_mesh(vertices, name):
    """Analyze mesh properties"""
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    bbox_size = bbox_max - bbox_min
    bbox_center = (bbox_min + bbox_max) / 2
    centroid = vertices.mean(axis=0)
    
    print(f"\n{'='*70}")
    print(f"{name}")
    print(f"{'='*70}")
    print(f"Vertices: {len(vertices)}")
    print(f"\nBounding Box:")
    print(f"  Min:    [{bbox_min[0]:10.6f}, {bbox_min[1]:10.6f}, {bbox_min[2]:10.6f}]")
    print(f"  Max:    [{bbox_max[0]:10.6f}, {bbox_max[1]:10.6f}, {bbox_max[2]:10.6f}]")
    print(f"  Size:   [{bbox_size[0]:10.6f}, {bbox_size[1]:10.6f}, {bbox_size[2]:10.6f}] m")
    print(f"  Size:   [{bbox_size[0]*1000:10.3f}, {bbox_size[1]*1000:10.3f}, {bbox_size[2]*1000:10.3f}] mm")
    print(f"  Center: [{bbox_center[0]:10.6f}, {bbox_center[1]:10.6f}, {bbox_center[2]:10.6f}]")
    print(f"\nCentroid: [{centroid[0]:10.6f}, {centroid[1]:10.6f}, {centroid[2]:10.6f}]")
    print(f"Max dimension: {bbox_size.max()*1000:.3f} mm")
    
    return bbox_center, bbox_size, centroid

def simulate_config_transform(vertices, max_size, target_position):
    """Simulate what the config does to mesh"""
    print(f"\n  Simulating config transform:")
    print(f"    max-size: {max_size}")
    print(f"    position: {target_position}")
    
    # Step 1: Center at origin (move to -centroid)
    centroid = vertices.mean(axis=0)
    vertices_centered = vertices - centroid
    print(f"    Step 1 - Center at origin (subtract centroid)")
    
    # Step 2: Scale to max_size
    bbox_size = vertices.max(axis=0) - vertices.min(axis=0)
    current_max = bbox_size.max()
    scale = max_size / current_max if current_max > 0 else 1.0
    vertices_scaled = vertices_centered * scale
    print(f"    Step 2 - Scale by {scale:.6f} (current_max={current_max*1000:.3f}mm -> target={max_size*1000:.3f}mm)")
    
    # Step 3: Move to target position
    vertices_final = vertices_scaled + np.array(target_position)
    print(f"    Step 3 - Move to position {target_position}")
    
    final_center = (vertices_final.min(axis=0) + vertices_final.max(axis=0)) / 2
    final_size = vertices_final.max(axis=0) - vertices_final.min(axis=0)
    
    print(f"  Final bbox center: [{final_center[0]:10.6f}, {final_center[1]:10.6f}, {final_center[2]:10.6f}]")
    print(f"  Final bbox size:   [{final_size[0]*1000:10.3f}, {final_size[1]*1000:10.3f}, {final_size[2]*1000:10.3f}] mm")
    
    return vertices_final, final_center

def main():
    print("="*70)
    print("MESH ALIGNMENT DIAGNOSIS")
    print("="*70)
    
    # Load meshes
    tumor_file = "resource/tissue/neuroma_tet_fixed.msh"
    bone_file = "resource/bone/refine05_fixed.obj"
    
    print(f"\nLoading tumor: {tumor_file}")
    tumor_verts = load_msh_surface(tumor_file)
    
    print(f"Loading bone: {bone_file}")
    bone_verts = load_obj(bone_file)
    
    # Analyze original meshes
    tumor_center, tumor_size, tumor_centroid = analyze_mesh(tumor_verts, "TUMOR (Original)")
    bone_center, bone_size, bone_centroid = analyze_mesh(bone_verts, "BONE (Original)")
    
    # Calculate distance
    distance = np.linalg.norm(tumor_centroid - bone_centroid)
    print(f"\n{'='*70}")
    print(f"Distance between centroids: {distance*1000:.3f} mm")
    print(f"{'='*70}")
    
    # Simulate config transforms
    print(f"\n{'='*70}")
    print("SIMULATING CONFIG TRANSFORMS")
    print(f"{'='*70}")
    
    print(f"\nTUMOR Transform:")
    tumor_final, tumor_final_center = simulate_config_transform(
        tumor_verts, max_size=0.05, target_position=[0, 0, 0]
    )
    
    print(f"\nBONE Transform:")
    bone_final, bone_final_center = simulate_config_transform(
        bone_verts, max_size=0.05, target_position=[0, 0, 0]
    )
    
    # Check overlap
    print(f"\n{'='*70}")
    print("OVERLAP CHECK")
    print(f"{'='*70}")
    
    distance_after = np.linalg.norm(tumor_final_center - bone_final_center)
    print(f"Distance between centers after transform: {distance_after*1000:.3f} mm")
    
    # Check if bounding boxes overlap
    tumor_min = tumor_final.min(axis=0)
    tumor_max = tumor_final.max(axis=0)
    bone_min = bone_final.min(axis=0)
    bone_max = bone_final.max(axis=0)
    
    overlap_x = not (tumor_max[0] < bone_min[0] or tumor_min[0] > bone_max[0])
    overlap_y = not (tumor_max[1] < bone_min[1] or tumor_min[1] > bone_max[1])
    overlap_z = not (tumor_max[2] < bone_min[2] or tumor_min[2] > bone_max[2])
    
    print(f"\nBounding box overlap:")
    print(f"  X-axis: {'✓ OVERLAP' if overlap_x else '✗ NO OVERLAP'}")
    print(f"  Y-axis: {'✓ OVERLAP' if overlap_y else '✗ NO OVERLAP'}")
    print(f"  Z-axis: {'✓ OVERLAP' if overlap_z else '✗ NO OVERLAP'}")
    
    if overlap_x and overlap_y and overlap_z:
        print(f"\n✓ Meshes SHOULD overlap after config transform")
    else:
        print(f"\n✗ Meshes DO NOT overlap after config transform")
        print(f"\nRECOMMENDATION:")
        offset = bone_final_center - tumor_final_center
        print(f"  Adjust bone position to: [{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}]")

if __name__ == '__main__':
    main()
