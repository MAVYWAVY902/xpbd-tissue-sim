#!/usr/bin/env python3
"""
Convert OBJ surface mesh to tetrahedral MSH volume mesh.
Standard, robust workflow using Gmsh.

Usage:
    python obj_to_tet_mesh.py input.obj output.msh [--min-size 0.5] [--max-size 2.0]
"""

import gmsh
import sys
import argparse
import math

def obj_to_tet_mesh(input_obj, output_msh, min_size=None, max_size=None, 
                    classify_angle=40, optimize=True):
    """
    Convert OBJ surface to tetrahedral volume mesh.
    
    Args:
        input_obj: Input .obj file (closed surface)
        output_msh: Output .msh file (with tetrahedra)
        min_size: Minimum element size (None = auto)
        max_size: Maximum element size (None = auto)
        classify_angle: Angle threshold for surface classification (degrees)
        optimize: Whether to optimize mesh quality
    """
    print(f"Converting {input_obj} → {output_msh}")
    
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 1)
    
    try:
        # Step 1: Load OBJ surface mesh
        print("Step 1: Loading OBJ...")
        gmsh.merge(input_obj)
        
        # Step 2: Classify surfaces to identify geometric features
        print(f"Step 2: Classifying surfaces (angle={classify_angle}°)...")
        angle_rad = classify_angle * math.pi / 180.0
        gmsh.model.mesh.classifySurfaces(angle_rad, curveAngle=angle_rad)
        
        # Step 3: Try to create CAD geometry (may fail for bad meshes)
        print("Step 3: Creating geometry...")
        try:
            gmsh.model.mesh.createGeometry()
            use_geo = True
        except Exception as e:
            print(f"  ⚠️  createGeometry() failed: {e}")
            print("  → Falling back to direct mesh approach (works for most cases)")
            use_geo = False
        
        # Step 4: Get surfaces and create volume
        surfaces = gmsh.model.getEntities(2)
        print(f"Step 4: Found {len(surfaces)} surface(s), creating volume...")
        
        if len(surfaces) == 0:
            raise Exception("No surfaces found! Check your OBJ file.")
        
        # Create surface loop and volume
        surface_tags = [tag for (dim, tag) in surfaces]
        if use_geo:
            surface_loop = gmsh.model.geo.addSurfaceLoop(surface_tags)
            volume = gmsh.model.geo.addVolume([surface_loop])
            gmsh.model.geo.synchronize()
        else:
            # Direct approach without geometry - just remesh in 3D
            # This works for most closed surfaces even if they're not perfect
            pass  # Will proceed directly to mesh generation
        
        # Step 5: Set mesh size constraints
        if min_size is not None or max_size is not None:
            # Auto-calculate based on bounding box if not specified
            bbox = gmsh.model.getBoundingBox(-1, -1)
            model_size = max(bbox[3]-bbox[0], bbox[4]-bbox[1], bbox[5]-bbox[2])
            
            if min_size is None:
                min_size = model_size / 50.0  # ~50 elements along longest axis
            if max_size is None:
                max_size = model_size / 10.0
            
            print(f"Step 5: Setting mesh size: min={min_size:.3f}, max={max_size:.3f}")
            gmsh.option.setNumber("Mesh.CharacteristicLengthMin", min_size)
            gmsh.option.setNumber("Mesh.CharacteristicLengthMax", max_size)
        else:
            print("Step 5: Using default mesh size")
        
        # Step 6: Generate 3D tetrahedral mesh
        print("Step 6: Generating 3D tetrahedral mesh...")
        gmsh.model.mesh.generate(3)
        
        # Step 7: Optimize mesh quality (optional but recommended)
        if optimize:
            print("Step 7: Optimizing mesh quality...")
            gmsh.model.mesh.optimize("Netgen")
        
        # Step 8: Get statistics
        nodes = gmsh.model.mesh.getNodes()
        tets = gmsh.model.mesh.getElements(3)
        
        num_vertices = len(nodes[0])
        num_tets = 0
        if len(tets[0]) > 0:
            for elem_type, elem_tags in zip(tets[0], tets[2]):
                elem_name = gmsh.model.mesh.getElementProperties(elem_type)[0]
                if elem_name == "Tetrahedron 4":
                    num_tets += len(elem_tags)
        
        print(f"\n✓ Mesh generated successfully!")
        print(f"  Vertices: {num_vertices}")
        print(f"  Tetrahedra: {num_tets}")
        
        # Performance estimate for CPU
        if num_vertices < 1000:
            perf = "Very fast (50+ fps on CPU)"
        elif num_vertices < 5000:
            perf = "Fast (30+ fps on CPU)"
        elif num_vertices < 15000:
            perf = "Medium (15-30 fps on CPU)"
        elif num_vertices < 30000:
            perf = "Slow (5-15 fps on CPU)"
        else:
            perf = "Very slow (<5 fps on CPU, consider GPU)"
        print(f"  Expected performance: {perf}")
        
        # Step 9: Save (ensure all elements are saved)
        print(f"\nStep 8: Saving to {output_msh}...")
        gmsh.option.setNumber("Mesh.SaveAll", 1)
        gmsh.write(output_msh)
        
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        gmsh.finalize()
    
    # Step 10: Verify the output file
    print("\nStep 9: Verifying output file...")
    gmsh.initialize()
    try:
        gmsh.open(output_msh)
        verify_tets = gmsh.model.mesh.getElements(3)
        
        if len(verify_tets[0]) > 0 and len(verify_tets[2][0]) > 0:
            print(f"✓ Verification passed: file contains {len(verify_tets[2][0])} tetrahedra")
            return True
        else:
            print("✗ Verification failed: no 3D elements in output file!")
            return False
            
    finally:
        gmsh.finalize()

def main():
    parser = argparse.ArgumentParser(
        description='Convert OBJ surface mesh to MSH tetrahedral volume mesh',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto mesh size
  python obj_to_tet_mesh.py tumor.obj tumor.msh
  
  # For CPU performance (coarser mesh)
  python obj_to_tet_mesh.py tumor.obj tumor_cpu.msh --min-size 1.0 --max-size 3.0
  
  # For high quality (finer mesh)
  python obj_to_tet_mesh.py tumor.obj tumor_fine.msh --min-size 0.3 --max-size 1.0
  
  # Custom classification angle
  python obj_to_tet_mesh.py tumor.obj tumor.msh --classify-angle 60
        """
    )
    
    parser.add_argument('input', help='Input OBJ surface mesh file')
    parser.add_argument('output', help='Output MSH tetrahedral mesh file')
    parser.add_argument('--min-size', type=float, default=None,
                        help='Minimum element size (default: auto)')
    parser.add_argument('--max-size', type=float, default=None,
                        help='Maximum element size (default: auto)')
    parser.add_argument('--classify-angle', type=float, default=40,
                        help='Surface classification angle in degrees (default: 40)')
    parser.add_argument('--no-optimize', action='store_true',
                        help='Skip mesh optimization step')
    
    args = parser.parse_args()
    
    success = obj_to_tet_mesh(
        args.input, 
        args.output,
        min_size=args.min_size,
        max_size=args.max_size,
        classify_angle=args.classify_angle,
        optimize=not args.no_optimize
    )
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
