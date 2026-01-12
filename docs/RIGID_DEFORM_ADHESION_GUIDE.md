# Rigid-Deform Adhesion Constraint Implementation Guide

## Overview

The `RigidDeformAdhesionConstraint` enables adhesion between rigid bodies and deformable meshes with breakable bonds. This is useful for simulating tissue adhesion to rigid surgical tools, bones, or implants.

## What Was Implemented

### 1. Core Constraint Class
**Files Created:**
- `include/solver/constraint/RigidDeformAdhesionConstraint.hpp`
- `src/solver/constraint/RigidDeformAdhesionConstraint.cpp`

**Key Features:**
- Inherits from both `Constraint` and `RigidBodyConstraint` for proper rigid body handling
- Uses `RigidBodyXPBDHelper` for correct rigid body position + orientation updates
- Implements tension-only adhesion (pulls when stretched, doesn't push)
- Strain-based breaking: bond breaks when `distance/rest_gap > break_ratio`
- Cached contact frame optimization for performance

**Physics:**
- Constraint function: `C(q) = max(0, separation - rest_gap)`
- Only active when rigid body and triangle are pulled apart beyond rest gap
- Gradient distributed across 3 triangle vertices; rigid body handled by helper

### 2. Integration with XPBD System
**Files Modified:**
- `include/common/XPBDTypedefs.hpp` - Added to type system
- `include/simobject/XPBDMeshObject.hpp` - Added creation method
- `src/simobject/XPBDMeshObject.cpp` - Implemented creation method
- `src/solver/CMakeLists.txt` - Added to build system

**Method Added:**
```cpp
addRigidDeformAdhesionConstraint(
    const Geometry::SDF* sdf,
    Sim::RigidObject* rigid_obj,
    const Vec3r& rigid_body_point,  // in body coordinates
    int tri_v1, int tri_v2, int tri_v3,
    Real rest_gap,
    Real break_ratio,
    Real alpha
)
```

### 3. Configuration System
**Files Modified:**
- `include/config/Config.hpp` - Added 5 new config parameters

**New Config Parameters:**
```yaml
rigid-deform-adhesion-enable: true/false      # Enable feature
rigid-deform-adhesion-rest-gap: 0.005         # Rest separation (m)
rigid-deform-adhesion-break-ratio: 2.0        # Break threshold
rigid-deform-adhesion-alpha: 1e-7             # Compliance
rigid-deform-adhesion-bond-distance: 0.015    # Creation threshold (m)
```

### 4. Simulation Integration
**Files Modified:**
- `src/simulation/Simulation.cpp` - Added constraint creation logic

**How It Works:**
1. Searches for rigid objects (name contains "RigidBone" or "Bone")
2. Searches for deformable objects (name contains "Tissue" or "DeformableMesh")
3. For each triangle face in deformable mesh:
   - Computes distance to rigid body
   - If `distance <= bond_distance`, creates adhesion constraint
   - Rigid body point is triangle centroid projected to body coordinates
4. Tries multiple XPBD type configurations (GaussSeidel/Jacobi variants)

## How to Use

### Step 1: Configure Your YAML File

```yaml
name: "My Rigid-Deform Test"
time-step: 0.001
end-time: 5.0

# Enable rigid-deform adhesion
rigid-deform-adhesion-enable: true
rigid-deform-adhesion-rest-gap: 0.005        # 5mm rest gap
rigid-deform-adhesion-break-ratio: 2.0       # Break at 100% strain
rigid-deform-adhesion-alpha: 1e-7            # Stiffness
rigid-deform-adhesion-bond-distance: 0.015   # 15mm creation radius

objects:
  - type: "rigid-mesh"
    name: "RigidBone"  # Name must match in Simulation.cpp
    mesh: "path/to/bone.obj"
    fixed: true
    collisions: true

  - type: "first-order-xpbd-mesh"
    name: "Tissue"  # Name must match in Simulation.cpp
    mesh: "path/to/tissue.obj"
    constraint-type: "StableNeohookean"
    youngs-modulus: 5000.0
```

### Step 2: Customize Object Names (if needed)

If your objects have different names, edit `Simulation.cpp` around line 2460:

```cpp
// Search for rigid objects
if (rigid_uptr->name() == "YourRigidObjectName") {
    rigid_obj_ptr = rigid_uptr.get();
}

// Search for deformable objects
if (fo_uptr->name() == "YourTissueName") {
    tissue_ptr = fo_uptr.get();
}
```

### Step 3: Build and Run

```bash
cd build
cmake ..
make
./YourExecutable config/rigid_deform_adhesion_test.yaml
```

## Key Differences from Inter-Deform Adhesion

| Feature | Inter-Deform | Rigid-Deform |
|---------|-------------|--------------|
| **Objects** | Two deformable meshes | Rigid body + deformable mesh |
| **Inheritance** | `Constraint` only | `Constraint` + `RigidBodyConstraint` |
| **DOF** | 4 vertices (12 coords) | 3 vertices + 1 rigid body (6 DOF) |
| **Projector** | `ConstraintProjector` | `RigidBodyConstraintProjector` |
| **Gradient** | Distributed across 4 vertices | 3 vertices + rigid body helper |
| **Rigid Body Point** | N/A | Stored in body coordinates |

## Parameters Guide

### `rest-gap`
- Initial separation distance at constraint creation
- Constraint activates when `current_distance > rest_gap`
- Typical: 0.001 - 0.01 m (1-10mm)

### `break-ratio`
- Strain threshold for breaking
- Formula: `bond_breaks = (max_distance / rest_gap) > break_ratio`
- Example: `2.0` means break at 100% strain (double the rest length)
- Typical: 1.5 - 3.0

### `alpha` (compliance)
- Controls constraint stiffness
- Lower = stiffer, higher = softer
- Typical: 1e-8 to 1e-6
- Too low: jittery simulation
- Too high: bonds don't hold

### `bond-distance`
- Maximum distance for creating bonds at initialization
- Only faces within this distance get adhesion constraints
- Should be larger than `rest-gap`
- Typical: 0.01 - 0.02 m (10-20mm)

## Debugging Tips

1. **No constraints created:**
   - Check object names match in Simulation.cpp
   - Verify `bond-distance` is large enough
   - Check console output for distance statistics

2. **Bonds too weak:**
   - Decrease `alpha` (more stiffness)
   - Increase solver iterations
   - Check `rest-gap` is appropriate

3. **Bonds too stiff/jittery:**
   - Increase `alpha` (more compliance)
   - Decrease time step
   - Check break ratio isn't too high

4. **Bonds break immediately:**
   - Increase `break-ratio`
   - Check initial geometry doesn't have pre-strain
   - Verify `rest-gap` matches initial distance

## Advanced Customization

### Custom Bond Creation Strategy

Currently bonds are created based on triangle centroid distance. To customize:

Edit `Simulation.cpp` around line 2495:
```cpp
// Current: Simple centroid distance
const Vec3r tri_center = (tri_p1 + tri_p2 + tri_p3) / 3.0;
const Real distance = (tri_center - rigid_pos).norm();

// Alternative 1: Use closest vertex
const Real d1 = (tri_p1 - rigid_pos).norm();
const Real d2 = (tri_p2 - rigid_pos).norm();
const Real d3 = (tri_p3 - rigid_pos).norm();
const Real distance = std::min({d1, d2, d3});

// Alternative 2: Use SDF for exact distance
if (sdf) {
    const Real distance = sdf->signedDistance(tri_center);
}
```

### Visualization

To visualize adhesion bonds, add vertex properties:
```cpp
// In constraint creation loop:
tissue_mesh->template addVertexProperty<bool>("has_rigid_adhesion", false);
auto& prop = tissue_mesh->template getVertexProperty<bool>("has_rigid_adhesion");
prop.set(v1, true);
prop.set(v2, true);
prop.set(v3, true);
```

## Next Steps

1. **Test with your geometry:**
   - Use example config as template
   - Adjust parameters for your use case
   - Monitor console output for diagnostics

2. **Tune for stability:**
   - Start with default parameters
   - Gradually increase stiffness (decrease alpha)
   - Balance with time step and solver iterations

3. **Extend functionality:**
   - Add custom bond creation patterns
   - Implement adaptive breaking based on stress
   - Add visualization of adhesion forces

## Implementation Complete! 🎉

All 6 steps completed:
✅ Header file created
✅ Implementation file created
✅ Type system updated
✅ XPBDMeshObject integration
✅ Simulation logic added
✅ CMakeLists.txt updated

The rigid-deform adhesion constraint is now fully integrated into your XPBD simulation system!
