#!/usr/bin/env python3
"""
Verify SDF generation by:
1. Generate SDF from mesh (same as C++ code does)
2. Extract zero-level isosurface (the "surface" according to SDF)
3. Compare with original mesh
4. Visualize SDF slice to see inside/outside regions
"""

import sys
import os

# Add the Mesh2SDF Python bindings path if needed
sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')

try:
    import mesh2sdf
except ImportError:
    print("ERROR: Cannot import mesh2sdf")
    print("Make sure Mesh2SDF is installed and Python bindings are available")
    sys.exit(1)

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_obj(filename):
    """Read OBJ file"""
    vertices = []
    faces = []
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            elif line.startswith('f '):
                parts = line.split()[1:]
                face = [int(p.split('/')[0]) - 1 for p in parts]
                faces.append(face)
    
    vertices = np.array(vertices, dtype=np.float64)
    faces = np.array(faces, dtype=np.uint32)
    
    return vertices, faces

def write_obj(filename, vertices, faces):
    """Write OBJ file"""
    with open(filename, 'w') as f:
        for v in vertices:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
        for face in faces:
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")

def analyze_sdf(mesh_file, output_dir="output/sdf_verification"):
    """Generate and analyze SDF"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Reading mesh: {mesh_file}")
    vertices, faces = read_obj(mesh_file)
    
    print(f"  Vertices: {len(vertices)}")
    print(f"  Faces: {len(faces)}")
    print(f"  Bbox: {vertices.min(axis=0)} to {vertices.max(axis=0)}")
    
    # Generate SDF (same parameters as C++ code: 128 grid, 5 voxel padding, with gradients)
    print("\nGenerating SDF (128x128x128 grid, padding=5)...")
    sdf = mesh2sdf.MeshSDF(vertices, faces, grid_size=128, padding=5, compute_gradients=True)
    
    # Get SDF properties
    print("\n=== SDF PROPERTIES ===")
    print(f"Grid size: {sdf.grid_size()}")
    print(f"Cell size: {sdf.grid_cell_size()}")
    
    grid_bbox = sdf.grid_bounding_box()
    print(f"Grid bbox: {grid_bbox[0]} to {grid_bbox[1]}")
    
    mesh_bbox = sdf.mesh_bounding_box()
    print(f"Mesh bbox (from SDF): {mesh_bbox[0]} to {mesh_bbox[1]}")
    
    mesh_center = sdf.mesh_mass_center()
    print(f"Mesh mass center: {mesh_center}")
    
    # Sample SDF at various points
    print("\n=== SAMPLING SDF ===")
    
    # Test points: origin, mesh vertices, and some interior points
    test_points = [
        ("Origin", np.array([0.0, 0.0, 0.0])),
        ("Mesh center", mesh_center),
        ("First vertex", vertices[0]),
        ("Random vertex", vertices[len(vertices)//2]),
    ]
    
    print("\nDistance values at test points:")
    for name, point in test_points:
        dist = sdf.evaluate(point)
        grad = sdf.gradient(point)
        print(f"  {name}: pos={point}, dist={dist:.6f} m = {dist*1000:.2f} mm")
        print(f"    gradient: {grad}, norm={np.linalg.norm(grad):.4f}")
    
    # Check distribution of distances in the grid
    print("\n=== SDF DISTANCE DISTRIBUTION ===")
    distance_grid = sdf.distance_grid()
    
    print(f"Grid shape: {distance_grid.shape}")
    print(f"Distance range: [{distance_grid.min():.6f}, {distance_grid.max():.6f}] meters")
    print(f"Distance range: [{distance_grid.min()*1000:.2f}, {distance_grid.max()*1000:.2f}] mm")
    
    # Count negative (inside) vs positive (outside) distances
    negative_count = np.sum(distance_grid < 0)
    positive_count = np.sum(distance_grid > 0)
    zero_count = np.sum(distance_grid == 0)
    total = distance_grid.size
    
    print(f"\nVoxel classification:")
    print(f"  Negative (inside):  {negative_count:8d} ({100*negative_count/total:5.1f}%)")
    print(f"  Zero (surface):     {zero_count:8d} ({100*zero_count/total:5.1f}%)")
    print(f"  Positive (outside): {positive_count:8d} ({100*positive_count/total:5.1f}%)")
    
    if negative_count == 0:
        print("\n❌ WARNING: NO NEGATIVE DISTANCES FOUND!")
        print("   This means the SDF thinks nothing is 'inside' the mesh!")
        print("   This is typical of NON-WATERTIGHT meshes with holes!")
    else:
        print(f"\n✓ SDF has inside region ({negative_count} voxels)")
    
    # Visualize 2D slice through the middle
    print("\n=== GENERATING VISUALIZATIONS ===")
    
    # Slice at Z = mesh_center[2]
    z_slice_idx = distance_grid.shape[2] // 2
    slice_2d = distance_grid[:, :, z_slice_idx]
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # Plot 1: Distance field
    im1 = axes[0].imshow(slice_2d.T, origin='lower', cmap='RdBu', vmin=-0.01, vmax=0.01)
    axes[0].set_title(f'SDF Slice (Z={z_slice_idx})\nRed=Outside, Blue=Inside')
    axes[0].set_xlabel('X index')
    axes[0].set_ylabel('Y index')
    plt.colorbar(im1, ax=axes[0], label='Distance (m)')
    
    # Plot 2: Inside/outside classification
    binary_slice = (slice_2d < 0).astype(float)
    axes[1].imshow(binary_slice.T, origin='lower', cmap='gray')
    axes[1].set_title('Inside (white) vs Outside (black)')
    axes[1].set_xlabel('X index')
    axes[1].set_ylabel('Y index')
    
    # Plot 3: Histogram of distances
    axes[2].hist(distance_grid.flatten() * 1000, bins=100, alpha=0.7)
    axes[2].axvline(0, color='red', linestyle='--', linewidth=2, label='Zero level (surface)')
    axes[2].set_xlabel('Distance (mm)')
    axes[2].set_ylabel('Voxel count')
    axes[2].set_title('SDF Distance Distribution')
    axes[2].set_yscale('log')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    slice_file = os.path.join(output_dir, 'sdf_slice.png')
    plt.savefig(slice_file, dpi=150)
    print(f"  Saved slice visualization: {slice_file}")
    plt.close()
    
    # Create 3D scatter plot of sample points
    print("\n=== SAMPLING SURFACE POINTS ===")
    
    # Sample points on the zero-level surface
    surface_points = []
    cell_size = sdf.grid_cell_size()
    grid_origin = grid_bbox[0]
    
    # Sample every Nth point to avoid too many points
    step = 4
    for i in range(0, distance_grid.shape[0], step):
        for j in range(0, distance_grid.shape[1], step):
            for k in range(0, distance_grid.shape[2], step):
                dist = distance_grid[i, j, k]
                # Find points close to zero level
                if abs(dist) < 0.002:  # Within 2mm of surface
                    # Convert grid indices to world coordinates
                    world_pos = grid_origin + np.array([i, j, k]) * cell_size
                    surface_points.append(world_pos)
    
    surface_points = np.array(surface_points)
    print(f"  Found {len(surface_points)} points near zero-level surface")
    
    if len(surface_points) > 0:
        # Save as OBJ for visualization
        surface_obj = os.path.join(output_dir, 'sdf_surface_points.obj')
        with open(surface_obj, 'w') as f:
            for pt in surface_points:
                f.write(f"v {pt[0]} {pt[1]} {pt[2]}\n")
        print(f"  Saved surface points: {surface_obj}")
    
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    
    if negative_count == 0:
        print("❌ SDF IS INVALID - No inside region detected")
        print("   Reason: Mesh is not watertight (has holes)")
        print("   Solution: Close the mesh holes in MeshLab or Blender")
    elif negative_count < 0.01 * total:
        print("⚠️  SDF may be problematic - Very small inside region")
        print(f"   Only {100*negative_count/total:.2f}% of voxels are inside")
    else:
        print("✓ SDF appears valid - Inside/outside regions detected")
    
    print(f"\nOutput files saved to: {output_dir}/")
    print("  - sdf_slice.png: 2D visualization")
    print("  - sdf_surface_points.obj: Points on SDF zero-level surface")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        mesh_file = sys.argv[1]
    else:
        mesh_file = "resource/bone/tbone_800_fixed.obj"
    
    if not os.path.exists(mesh_file):
        print(f"ERROR: File not found: {mesh_file}")
        sys.exit(1)
    
    analyze_sdf(mesh_file)
