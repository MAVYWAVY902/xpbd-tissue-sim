#!/usr/bin/env python3
"""
Analyze SDF reconstruction to find artifacts
"""
import sys
import numpy as np
from collections import defaultdict

def load_obj(filename):
    vertices = []
    faces = []
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith('f '):
                parts = line.split()[1:]
                face = []
                for part in parts:
                    v_idx = int(part.split('/')[0]) - 1
                    face.append(v_idx)
                faces.append(face)
    
    return np.array(vertices), faces

def analyze_mesh(vertices, faces, name):
    print(f"\n{'='*70}")
    print(f"Analyzing: {name}")
    print(f"{'='*70}")
    
    print(f"Vertices: {len(vertices)}")
    print(f"Faces: {len(faces)}")
    
    # Bounding box
    bbox_min = vertices.min(axis=0)
    bbox_max = vertices.max(axis=0)
    bbox_size = bbox_max - bbox_min
    bbox_center = (bbox_min + bbox_max) / 2
    
    print(f"\nBounding box:")
    print(f"  Min: {bbox_min}")
    print(f"  Max: {bbox_max}")
    print(f"  Size: {bbox_size} m = {bbox_size*1000} mm")
    print(f"  Center: {bbox_center}")
    
    # Find connected components
    print(f"\nAnalyzing connectivity...")
    
    # Build adjacency
    vertex_faces = defaultdict(list)
    for face_idx, face in enumerate(faces):
        for v in face:
            vertex_faces[v].append(face_idx)
    
    # Find connected components via BFS
    visited_faces = set()
    components = []
    
    for start_face in range(len(faces)):
        if start_face in visited_faces:
            continue
            
        # BFS from this face
        component = []
        queue = [start_face]
        visited_faces.add(start_face)
        
        while queue:
            face_idx = queue.pop(0)
            component.append(face_idx)
            
            # Find neighbors (faces sharing vertices)
            for v in faces[face_idx]:
                for neighbor_face in vertex_faces[v]:
                    if neighbor_face not in visited_faces:
                        visited_faces.add(neighbor_face)
                        queue.append(neighbor_face)
        
        components.append(component)
    
    # Sort by size
    components.sort(key=len, reverse=True)
    
    print(f"\n🔍 Found {len(components)} connected components:")
    for i, comp in enumerate(components[:10]):  # Show top 10
        comp_vertices = set()
        for face_idx in comp:
            comp_vertices.update(faces[face_idx])
        
        # Get bounding box of this component
        comp_verts = vertices[list(comp_vertices)]
        comp_bbox_min = comp_verts.min(axis=0)
        comp_bbox_max = comp_verts.max(axis=0)
        comp_size = comp_bbox_max - comp_bbox_min
        
        percentage = 100 * len(comp) / len(faces)
        print(f"  Component {i+1}: {len(comp)} faces ({percentage:.1f}%), "
              f"{len(comp_vertices)} verts, size={comp_size*1000} mm")
    
    # Check for small isolated components (likely artifacts)
    main_component_size = len(components[0])
    artifact_threshold = 0.01 * main_component_size  # Components <1% of main
    
    artifacts = [c for c in components[1:] if len(c) < artifact_threshold]
    
    if artifacts:
        print(f"\n⚠️  Found {len(artifacts)} small components (likely artifacts):")
        print(f"     These make up {sum(len(c) for c in artifacts)} faces ({100*sum(len(c) for c in artifacts)/len(faces):.2f}% of total)")
        print(f"\n💡 Solution: Filter out components smaller than {artifact_threshold:.0f} faces")
    else:
        print(f"\n✓ No small artifact components found")
    
    return components

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_sdf_reconstruction.py <reconstructed.obj> [original.obj]")
        sys.exit(1)
    
    reconstructed_file = sys.argv[1]
    
    # Load reconstructed mesh
    recon_verts, recon_faces = load_obj(reconstructed_file)
    components = analyze_mesh(recon_verts, recon_faces, "Reconstructed mesh")
    
    # Optionally compare with original
    if len(sys.argv) > 2:
        original_file = sys.argv[2]
        orig_verts, orig_faces = load_obj(original_file)
        analyze_mesh(orig_verts, orig_faces, "Original mesh")
        
        # Compare sizes
        print(f"\n{'='*70}")
        print("Comparison:")
        print(f"{'='*70}")
        print(f"Vertex count: {len(orig_verts)} → {len(recon_verts)} "
              f"({100*len(recon_verts)/len(orig_verts):.1f}%)")
        print(f"Face count: {len(orig_faces)} → {len(recon_faces)} "
              f"({100*len(recon_faces)/len(orig_faces):.1f}%)")
    
    # Recommendation
    print(f"\n{'='*70}")
    print("RECOMMENDATION:")
    print(f"{'='*70}")
    if len(components) > 1:
        main_size = len(components[0])
        total_size = sum(len(c) for c in components)
        artifact_ratio = (total_size - main_size) / total_size
        
        if artifact_ratio > 0.01:  # More than 1% artifacts
            print(f"❌ Mesh has {len(components)-1} extra components ({artifact_ratio*100:.1f}% of faces)")
            print(f"   These are likely SDF artifacts from:")
            print(f"   - Grid boundary issues")
            print(f"   - Non-watertight original mesh")
            print(f"   - Numerical errors in marching cubes")
            print(f"\n   FIX: Filter out components < {0.01*main_size:.0f} faces")
        else:
            print(f"✓ Mesh is clean (only {artifact_ratio*100:.2f}% artifacts)")
    else:
        print(f"✓ Mesh is a single connected component (ideal)")

if __name__ == '__main__':
    main()
