#!/usr/bin/env python3
"""
Quick test to verify the fixed mesh loads correctly
"""
import sys

try:
    import meshio
    
    print("Testing neuroma_tet_fixed.msh...")
    mesh = meshio.read("resource/tissue/neuroma_tet_fixed.msh")
    
    print(f"✅ Mesh loaded successfully!")
    print(f"  Nodes: {len(mesh.points)}")
    print(f"  Elements: {sum(len(cells.data) for cells in mesh.cells)}")
    print(f"  Coordinate range:")
    print(f"    X: [{mesh.points[:, 0].min():.6f}, {mesh.points[:, 0].max():.6f}]")
    print(f"    Y: [{mesh.points[:, 1].min():.6f}, {mesh.points[:, 1].max():.6f}]")
    print(f"    Z: [{mesh.points[:, 2].min():.6f}, {mesh.points[:, 2].max():.6f}]")
    
    # Check no duplicates
    unique_points = len(set(tuple(p) for p in mesh.points))
    if unique_points == len(mesh.points):
        print(f"✅ All {len(mesh.points)} nodes are unique (no duplicates)")
    else:
        print(f"❌ Found duplicate nodes: {len(mesh.points) - unique_points} duplicates")
        sys.exit(1)
    
    print("\n✅ Mesh file is valid and ready for simulation!")
    
except ImportError:
    print("meshio not installed, trying basic file check...")
    with open("resource/tissue/neuroma_tet_fixed.msh", 'r') as f:
        content = f.read()
        if "$Entities" in content and "$Nodes" in content and "$Elements" in content:
            print("✅ File has correct GMSH structure")
        else:
            print("❌ File missing required sections")
            sys.exit(1)
    
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
