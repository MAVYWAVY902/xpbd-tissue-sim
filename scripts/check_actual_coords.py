#!/usr/bin/env python3
"""
Check what coordinates the meshes actually have when loaded
"""
import numpy as np

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
    i += 1
    
    for block in range(num_entity_blocks):
        entity_info = lines[i].strip().split()
        num_nodes_in_block = int(entity_info[3])
        i += 1
        
        # Read node tags
        for _ in range(num_nodes_in_block):
            i += 1
        
        # Read coordinates
        for _ in range(num_nodes_in_block):
            coords = list(map(float, lines[i].strip().split()))
            vertices.append(coords)
            i += 1
    
    return np.array(vertices)

def analyze(vertices, name):
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    bbox_center = (bbox_min + bbox_max) / 2
    bbox_size = bbox_max - bbox_min
    
    print(f"\n{'='*70}")
    print(f"{name}")
    print(f"{'='*70}")
    print(f"Vertices: {len(vertices)}")
    print(f"BBox Min:    [{bbox_min[0]:12.6f}, {bbox_min[1]:12.6f}, {bbox_min[2]:12.6f}]")
    print(f"BBox Max:    [{bbox_max[0]:12.6f}, {bbox_max[1]:12.6f}, {bbox_max[2]:12.6f}]")
    print(f"BBox Center: [{bbox_center[0]:12.6f}, {bbox_center[1]:12.6f}, {bbox_center[2]:12.6f}]")
    print(f"BBox Size:   [{bbox_size[0]:12.6f}, {bbox_size[1]:12.6f}, {bbox_size[2]:12.6f}] m")
    print(f"BBox Size:   [{bbox_size[0]*1000:12.3f}, {bbox_size[1]*1000:12.3f}, {bbox_size[2]*1000:12.3f}] mm")
    
    return bbox_center

# Load tumor
print("Loading meshes from config...")
tumor_verts = load_msh_surface("resource/tissue/neuroma_tet_fixed.msh")
bone_verts = load_obj("resource/tbone_fixed/tbone_ds06.obj")

tumor_center = analyze(tumor_verts, "TUMOR (as stored in file)")
bone_center = analyze(bone_verts, "BONE (as stored in file)")

distance = np.linalg.norm(tumor_center - bone_center)
offset = bone_center - tumor_center

print(f"\n{'='*70}")
print(f"ALIGNMENT CHECK")
print(f"{'='*70}")
print(f"Distance between centers: {distance*1000:.3f} mm")
print(f"Offset (bone - tumor):    [{offset[0]:12.6f}, {offset[1]:12.6f}, {offset[2]:12.6f}]")
print(f"Offset (bone - tumor):    [{offset[0]*1000:12.3f}, {offset[1]*1000:12.3f}, {offset[2]*1000:12.3f}] mm")

if distance < 0.001:  # < 1mm
    print(f"\n✓ Centers are aligned (distance < 1mm)")
else:
    print(f"\n✗ Centers are NOT aligned (distance = {distance*1000:.3f} mm)")
    print(f"\nWith use-original-coords: true, simulation will load these exact coordinates")
    print(f"To fix alignment in simulation, adjust bone position by offset:")
    print(f"  position: [{-offset[0]:.6f}, {-offset[1]:.6f}, {-offset[2]:.6f}]")
