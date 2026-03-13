#!/usr/bin/env python3
"""
Verify SDF by loading binary dump from C++ and visualizing it.

Usage:
    python3 scripts/verify_sdf.py sdf_verification/knife_sdf_dump.bin

Binary file format (written by Geometry::MeshSDF::dumpToFile):
    - 3 x int32:  ni, nj, nk (grid dimensions)
    - 3 x float64: cell_size_x, cell_size_y, cell_size_z
    - 3 x float64: bbox_min_x, bbox_min_y, bbox_min_z
    - 3 x float64: bbox_max_x, bbox_max_y, bbox_max_z
    - ni*nj*nk x float64: distance values (flat array)
"""

import sys
import os
import struct

import numpy as np
import matplotlib.pyplot as plt


def load_sdf_dump(filename):
    """Load binary SDF dump file."""
    with open(filename, "rb") as f:
        # Grid dimensions: 3 x int32
        ni, nj, nk = struct.unpack("iii", f.read(12))

        # Cell size: 3 x float64
        cell_size = np.array(struct.unpack("ddd", f.read(24)))

        # Bounding box min: 3 x float64
        bbox_min = np.array(struct.unpack("ddd", f.read(24)))

        # Bounding box max: 3 x float64
        bbox_max = np.array(struct.unpack("ddd", f.read(24)))

        # Distance grid: ni*nj*nk x float64
        num_values = ni * nj * nk
        data = np.frombuffer(f.read(num_values * 8), dtype=np.float64)
        # Array3 stores data in Fortran/column-major order (i varies fastest)
        # i.e. Array3(i,j,k) = data[i + ni*j + ni*nj*k]
        grid = data.reshape((ni, nj, nk), order='F')

    return grid, cell_size, bbox_min, bbox_max


def analyze_sdf(grid, cell_size, bbox_min, bbox_max, output_dir):
    """Analyze and print SDF statistics."""
    ni, nj, nk = grid.shape

    print(f"{'='*60}")
    print("SDF GRID PROPERTIES (from C++ dump)")
    print(f"{'='*60}")
    print(f"Grid dimensions: {ni} x {nj} x {nk}")
    print(f"Cell size:       ({cell_size[0]:.6f}, {cell_size[1]:.6f}, {cell_size[2]:.6f})")
    print(f"Cell size (mm):  ({cell_size[0]*1000:.4f}, {cell_size[1]*1000:.4f}, {cell_size[2]*1000:.4f})")
    print(f"Bbox min:        ({bbox_min[0]:.6f}, {bbox_min[1]:.6f}, {bbox_min[2]:.6f})")
    print(f"Bbox max:        ({bbox_max[0]:.6f}, {bbox_max[1]:.6f}, {bbox_max[2]:.6f})")
    bbox_size = bbox_max - bbox_min
    print(f"Bbox size:       ({bbox_size[0]:.6f}, {bbox_size[1]:.6f}, {bbox_size[2]:.6f})")
    print(f"Bbox size (mm):  ({bbox_size[0]*1000:.4f}, {bbox_size[1]*1000:.4f}, {bbox_size[2]*1000:.4f})")

    print(f"\n{'='*60}")
    print("SDF DISTANCE DISTRIBUTION")
    print(f"{'='*60}")
    print(f"Distance range: [{grid.min():.6f}, {grid.max():.6f}]")
    print(f"Distance range (mm): [{grid.min()*1000:.4f}, {grid.max()*1000:.4f}]")

    negative_count = int(np.sum(grid < 0))
    positive_count = int(np.sum(grid > 0))
    zero_count = int(np.sum(grid == 0))
    total = grid.size

    print(f"\nVoxel classification:")
    print(f"  Negative (inside):  {negative_count:8d} ({100*negative_count/total:5.1f}%)")
    print(f"  Zero (surface):     {zero_count:8d} ({100*zero_count/total:5.1f}%)")
    print(f"  Positive (outside): {positive_count:8d} ({100*positive_count/total:5.1f}%)")

    if negative_count == 0:
        print("\n  WARNING: NO NEGATIVE DISTANCES FOUND!")
        print("  The SDF has no inside region. Mesh may not be watertight.")
    else:
        print(f"\n  SDF has inside region ({negative_count} voxels)")


def visualize_slices(grid, output_dir):
    """Generate orthogonal slice visualizations."""
    ni, nj, nk = grid.shape

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    slices = [
        ("X (i)", ni // 2, grid[ni // 2, :, :]),
        ("Y (j)", nj // 2, grid[:, nj // 2, :]),
        ("Z (k)", nk // 2, grid[:, :, nk // 2]),
    ]

    vmax = min(abs(grid.min()), abs(grid.max()), 0.5)

    for col, (axis_name, idx, slice_2d) in enumerate(slices):
        # Row 0: Distance field with diverging colormap
        im = axes[0, col].imshow(
            slice_2d.T, origin="lower", cmap="RdBu",
            vmin=-vmax, vmax=vmax,
        )
        axes[0, col].set_title(f"SDF Slice ({axis_name}={idx})\nRed=Outside, Blue=Inside")
        plt.colorbar(im, ax=axes[0, col], label="Distance")

        # Row 1: Inside/outside binary
        binary_slice = (slice_2d < 0).astype(float)
        axes[1, col].imshow(binary_slice.T, origin="lower", cmap="gray")
        axes[1, col].set_title("Inside (white) / Outside (black)")

    plt.tight_layout()
    slice_file = os.path.join(output_dir, "sdf_slices.png")
    plt.savefig(slice_file, dpi=150)
    print(f"  Saved slice visualization: {slice_file}")
    plt.close()


def visualize_histogram(grid, output_dir):
    """Generate distance histogram."""
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(grid.flatten() * 1000, bins=200, alpha=0.7)
    ax.axvline(0, color="red", linestyle="--", linewidth=2, label="Zero level (surface)")
    ax.set_xlabel("Distance (mm)")
    ax.set_ylabel("Voxel count")
    ax.set_title("SDF Distance Distribution")
    ax.set_yscale("log")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    hist_file = os.path.join(output_dir, "sdf_histogram.png")
    plt.savefig(hist_file, dpi=150)
    print(f"  Saved histogram: {hist_file}")
    plt.close()


def reconstruct_surface(grid, cell_size, bbox_min, output_dir):
    """Use marching cubes to extract zero-level isosurface and export as .obj."""
    try:
        from skimage.measure import marching_cubes
    except ImportError:
        print("  WARNING: scikit-image not installed. Skipping marching cubes.")
        print("  Install with: pip3 install scikit-image")
        return

    ni, nj, nk = grid.shape

    # Check that there's an inside region
    if np.all(grid >= 0) or np.all(grid <= 0):
        print("  WARNING: SDF has no sign change — cannot extract zero-level surface.")
        return

    print(f"  Running marching cubes on {ni}x{nj}x{nk} grid...")
    verts, faces, normals, values = marching_cubes(grid, level=0.0, spacing=cell_size)

    # Shift vertices to world coordinates (marching cubes outputs in grid-local coords)
    verts += bbox_min

    print(f"  Extracted {len(verts)} vertices, {len(faces)} triangles")

    # Write OBJ
    obj_file = os.path.join(output_dir, "reconstructed_surface.obj")
    with open(obj_file, "w") as f:
        f.write(f"# Reconstructed from SDF zero-level isosurface via marching cubes\n")
        f.write(f"# {len(verts)} vertices, {len(faces)} faces\n")
        for v in verts:
            f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")
        for n in normals:
            f.write(f"vn {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n")
        for face in faces:
            # OBJ is 1-indexed
            f.write(f"f {face[0]+1}//{face[0]+1} {face[1]+1}//{face[1]+1} {face[2]+1}//{face[2]+1}\n")

    print(f"  Saved reconstructed surface: {obj_file}")
    print(f"  Compare with original mesh in MeshLab:")
    print(f"    meshlab resource/tools/dissector_uv.obj {obj_file}")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <sdf_dump.bin> [output_dir]")
        print(f"  sdf_dump.bin: Binary dump from MeshSDF::dumpToFile()")
        print(f"  output_dir:   Output directory (default: same dir as input)")
        sys.exit(1)

    dump_file = sys.argv[1]
    if not os.path.exists(dump_file):
        print(f"ERROR: File not found: {dump_file}")
        sys.exit(1)

    output_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.dirname(dump_file) or "."
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading SDF dump: {dump_file}")
    grid, cell_size, bbox_min, bbox_max = load_sdf_dump(dump_file)

    analyze_sdf(grid, cell_size, bbox_min, bbox_max, output_dir)

    print(f"\n{'='*60}")
    print("GENERATING VISUALIZATIONS")
    print(f"{'='*60}")
    visualize_slices(grid, output_dir)
    visualize_histogram(grid, output_dir)

    print(f"\n{'='*60}")
    print("RECONSTRUCTING SURFACE (marching cubes)")
    print(f"{'='*60}")
    reconstruct_surface(grid, cell_size, bbox_min, output_dir)

    print(f"\n{'='*60}")
    print("DONE")
    print(f"{'='*60}")
    print(f"Output files in: {output_dir}/")
    print(f"  - sdf_slices.png:            2D slice visualization")
    print(f"  - sdf_histogram.png:         Distance distribution histogram")
    print(f"  - reconstructed_surface.obj: Marching cubes zero-level surface")


if __name__ == "__main__":
    main()
