# BUG #10: Collision-Adhesion Constraint Conflict

## Problem Description

Rigid-deform collision constraints are **severely jiggling and penetrating** when rigid-deform adhesion is enabled.

## User Symptoms

1. **Jiggling**: The tumor oscillates/vibrates near the bone surface
2. **Penetration**: The tumor penetrates into the bone despite collision constraints
3. **Instability**: Behavior does not improve with parameter tuning

## Root Cause Analysis

### The Fatal Flaw: Opposing Forces on Same Geometry

When both collision AND adhesion constraints are active on the **same triangles**, they create **directly opposing forces**:

```
Collision Constraint: C = penetration_depth (negative when penetrating)
    → Pushes triangle AWAY from bone surface (outward)
    → Force direction: +normal (away from bone)

Adhesion Constraint: C = max(0, distance - rest_gap)
    → Pulls triangle TOWARD bone surface (inward)
    → Force direction: -normal (toward bone)
```

### Scenario Timeline

#### Initial State (t=0)
```
Tumor faces at 18.9-46.1mm from bone
All 1500 faces get adhesion constraints (bond_distance=50mm)
rest_gap = 0.5mm
C_adhesion = distance - 0.5mm = 18.4-45.6mm (HUGE positive value)
```

#### Adhesion Phase (t=0 to t=2s)
```
Adhesion pulls tumor toward bone
For face at 20mm distance:
  C = 19.5mm
  dlam ≈ -19.5 / (weight + alpha_tilde)
  → Triangle moves TOWARD bone
```

#### Collision Detection (t≈2s)
```
Tumor surface approaches within 1µm of bone
Collision constraint created (threshold = 1e-6 meters)
penetration_depth ≈ -0.0001mm (tiny penetration)
```

#### **CONFLICT ZONE** (t>2s) ⚠️

**SAME triangle now has BOTH constraints:**

```
ADHESION CONSTRAINT:
  distance = 0.0001mm (slightly penetrated)
  C_adhesion = max(0, 0.0001 - 0.5) = 0 (inactive? NO!)
  Actually: distance measured to SURFACE, not penetration
  C_adhesion ≈ 0.4999mm (still trying to pull inward!)
  
COLLISION CONSTRAINT:
  penetration = -0.0001mm
  C_collision = -0.0001mm (pushing outward!)
```

**Result**: The two constraints **fight each other**:
- Adhesion: "Pull toward bone!" → Force = -500 × normal
- Collision: "Push away from bone!" → Force = +100 × normal
- Net effect: **Oscillation!** 

The triangle gets pulled in → penetrates → pushed out → repeats forever!

### Why Parameters Don't Fix This

No matter what parameters you choose:

1. **Increase collision alpha**: Makes collision softer → more penetration
2. **Decrease collision alpha**: Makes collision stiffer → more jiggling
3. **Increase adhesion rest_gap**: Doesn't help - still pulling when touching
4. **Decrease adhesion alpha**: Makes adhesion weaker → tissue falls away
5. **Increase adhesion alpha**: Makes adhesion softer → more penetration

**The fundamental problem is LOGICAL, not numerical!**

## Detailed Mechanism Analysis

### Collision Constraint (RigidDeformableCollisionConstraint)

**Trigger Condition**:
```cpp
// From CollisionScene.cpp L690
const double distance = sdf->evaluate(x);
if (distance <= 1e-6)  // 1 micrometer penetration threshold
{
    xpbd_mesh_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, i, u, v, w);
}
```

**Constraint Function**:
```cpp
// From RigidDeformableCollisionConstraint.cpp L30
void evaluate(Real* C) const
{
    const Vec3r a = _u*p1 + _v*p2 + _w*p3;  // Barycentric interpolation
    *C = _sdf->evaluate(a);  // Penetration depth (negative = penetrating)
}
```

**Force Direction**:
```cpp
// gradient() returns collision_normal * barycentric_weight
// collision_normal points AWAY from bone surface (outward)
// When C < 0 (penetrating), dlam < 0
// Update: position += dlam * gradient = position - |dlam| * collision_normal
// Result: Pushed AWAY from bone ✅
```

**Inequality Constraint**: Only enforced when `C <= 0` (penetrating)

### Adhesion Constraint (RigidDeformAdhesionConstraint)

**Trigger Condition**:
```cpp
// From Simulation.cpp L2600+
// For each face centroid, check distance to rigid surface
const Real face_to_surface_distance = /* computed via SDF */;
if (face_to_surface_distance < bond_distance) {  // 50mm
    // Create adhesion constraint
}
```

**Constraint Function**:
```cpp
// From RigidDeformAdhesionConstraint.cpp L55
void evaluate(Real* C) const
{
    // Compute closest point on triangle to rigid point
    const Real separation_distance = computePointTriangleDistance(...);
    
    // Single-sided spring: only pulls when separated beyond rest_gap
    *C = std::max(0.0, separation_distance - _rest_gap);
}
```

**Force Direction**:
```cpp
// gradient() returns normal * barycentric_weight
// normal points FROM triangle TO rigid point (inward when close to surface!)
// When C > 0 (separated), dlam < 0
// Update: position += dlam * gradient = position - |dlam| * normal
// Result: Pulled TOWARD bone ✅
```

**Inequality Constraint**: Only enforced when `C > 0` (separated beyond rest_gap)

### The Conflict Zone

**When do BOTH constraints activate simultaneously?**

```
Let's say face is at distance d from bone surface:

COLLISION: Active when d <= 1µm (practically touching/penetrating)
ADHESION: Active when d > rest_gap (separated more than rest_gap)

CONFLICT when: d <= 1µm AND d > rest_gap
This requires: rest_gap < 1µm

But our rest_gap = 0.5mm = 500µm !!!
```

**WAIT... Let me reconsider the adhesion constraint evaluation!**

Looking more carefully at the code:

```cpp
// Adhesion measures distance from RIGID POINT to TRIANGLE SURFACE
// NOT the SDF distance (which is triangle point to rigid surface)

// When triangle is TOUCHING the rigid surface:
// - Collision: measures penetration (triangle points inside SDF)
// - Adhesion: measures rigid-point-to-triangle distance

// These are DIFFERENT distances!!
// If rigid point is at bone surface, and triangle is touching:
//   - Collision distance: ≈ 0 (touching/slightly penetrating)
//   - Adhesion distance: Could be 5-20mm (rigid point to triangle)
//   → Adhesion still pulls! C = 19.5mm for 20mm distance
```

## The Real Bug

The adhesion constraint uses **rigid point to triangle closest point** distance, which can be LARGE even when the triangle is touching the bone surface!

**Example**:
```
Bone is a large mesh object
Triangle A is at position (10, 0, 0), touching bone surface at that location
Rigid body "point" for this adhesion bond is at bone position (30, 0, 0)

Collision:
  Triangle A penetration = -0.0001mm → Push outward ✅

Adhesion:
  Distance from (30,0,0) to triangle A = 20mm
  C = 20 - 0.5 = 19.5mm → Pull inward! ❌

RESULT: Conflict! Adhesion pulls toward (30,0,0), collision pushes away from (10,0,0)
```

## Solution Approaches

### Option 1: Disable Collision When Adhesion Active (RECOMMENDED)

**Logic**: If adhesion is responsible for keeping tissues together, let it also handle contact/penetration.

```cpp
// In CollisionScene.cpp, before creating collision constraint:

// Check if this face already has an adhesion constraint
bool has_adhesion = xpbd_mesh_obj->faceHasRigidDeformAdhesion(face_index, rigid_obj);

if (!has_adhesion) {
    xpbd_mesh_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, i, u, v, w);
}
```

**Pros**:
- Eliminates conflict completely
- Adhesion handles both attraction AND contact
- Simpler system (fewer constraints)

**Cons**:
- Adhesion must be strong enough to prevent penetration
- May need smaller rest_gap and larger alpha for stability

### Option 2: Use Two-Sided Adhesion Constraint

**Logic**: Make adhesion both attractive AND repulsive, replacing collision.

```cpp
// In RigidDeformAdhesionConstraint::evaluate()
void evaluate(Real* C) const
{
    const Real separation_distance = computePointTriangleDistance(...);
    
    // Two-sided spring: penalty for both separation AND penetration
    *C = separation_distance - _rest_gap;  // Remove max(0, ...)
    
    // When C > 0: tissue too far, pull together
    // When C < 0: tissue too close, push apart
}
```

**Pros**:
- Single unified constraint for adhesion + collision
- Physically realistic spring behavior
- Avoids constraint conflicts

**Cons**:
- Changes adhesion semantics (now also repulsive)
- May need careful tuning of rest_gap (equilibrium distance)
- Could cause issues if rest_gap != actual desired contact distance

### Option 3: Collision Priority + Adhesion Deactivation

**Logic**: When collision is active, temporarily deactivate adhesion on that face.

```cpp
// In RigidDeformAdhesionConstraint::evaluate()
void evaluate(Real* C) const
{
    // Check if this face has active collision constraint
    bool has_collision = /* query collision constraint list */;
    
    if (has_collision) {
        *C = 0;  // Deactivate adhesion, let collision handle it
        return;
    }
    
    // Normal adhesion evaluation
    const Real separation_distance = computePointTriangleDistance(...);
    *C = std::max(0.0, separation_distance - _rest_gap);
}
```

**Pros**:
- Collision has priority (prevents penetration)
- Adhesion only acts when not in contact
- Clear separation of responsibilities

**Cons**:
- Complex interaction between systems
- Requires cross-constraint communication
- May cause discontinuities when switching

### Option 4: Distance-Based Adhesion Rest Gap

**Logic**: Set rest_gap equal to the collision threshold, so adhesion deactivates at contact.

```
rest_gap = 1e-6  // 1 micrometer (collision threshold)
```

**Pros**:
- Simple parameter change
- Clear separation: adhesion far, collision near

**Cons**:
- **DOESN'T WORK!** Adhesion distance != collision distance (different measurement points)
- Adhesion would almost always be active (19.999mm > 0.001mm)
- Doesn't solve the fundamental measurement mismatch

## Recommended Fix: Option 1 + Modified Collision Creation

**Implementation**:

1. **Add method to check if face has adhesion**:

```cpp
// In XPBDMeshObject.hpp
bool faceHasRigidDeformAdhesion(int face_index, Sim::RigidObject* rigid_obj) const;
```

2. **Modify collision creation**:

```cpp
// In CollisionScene.cpp L700
if (distance <= 1e-6)
{
    // Check if adhesion already handles this face-rigid pair
    bool has_adhesion = _config->rigidDeformAdhesionEnable() && 
                       xpbd_mesh_obj->faceHasRigidDeformAdhesion(i, rigid_obj);
    
    if (has_adhesion) {
        // Let adhesion handle both attraction and contact
        // Only create collision if adhesion is disabled
        continue;
    }
    
    if (rigid_obj->isFixed())
    {
        xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
    }
    else
    {
        xpbd_mesh_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, i, u, v, w);
    }
}
```

3. **Ensure adhesion prevents penetration**:
   - Use small rest_gap (e.g., 0.5mm)
   - Use moderate alpha for stability (e.g., 1e-4 to 1e-5)
   - The adhesion constraint will act as both adhesive bond AND soft collision

## Testing Plan

1. **Verify constraint counts**:
   - Before fix: ~1500 adhesion + ~50-100 collision per frame
   - After fix: ~1500 adhesion, 0 collision (on adhered faces)

2. **Test penetration**:
   - Apply force to tumor toward bone
   - Verify: No penetration (adhesion prevents it)

3. **Test adhesion strength**:
   - Grasp and pull tumor away from bone
   - Verify: Bonds stretch but don't break easily

4. **Test stability**:
   - Run simulation for 10+ seconds
   - Verify: No jiggling, smooth motion

## Summary

**Bug #10: Collision-Adhesion Constraint Conflict**

- **Root cause**: Collision and adhesion create opposing forces on same triangles
- **Why it happens**: Adhesion measures rigid-point-to-triangle distance (large), collision measures triangle-point-to-surface distance (small)
- **Why parameters don't fix it**: Logical conflict, not numerical
- **Recommended fix**: Disable collision creation on faces with active adhesion
- **Result**: Single constraint handles both adhesion and contact, no conflicts
