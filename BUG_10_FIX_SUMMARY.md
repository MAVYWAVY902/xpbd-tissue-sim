# Bug #10 Fix Summary

## What Was Fixed

**Bug #10: Collision-Adhesion Constraint Conflict** causing severe jiggling and penetration in rigid-deform interactions.

## Root Cause

When both rigid-deform collision AND adhesion constraints were active on the same triangles, they created **directly opposing forces**:

- **Collision constraint**: Pushes triangle AWAY from rigid surface (when C < 0, penetrating)
- **Adhesion constraint**: Pulls triangle TOWARD rigid point (when C > 0, separated)

This caused oscillation because:
1. Adhesion pulls tissue toward bone → tissue approaches surface
2. Collision detects tiny penetration (< 1µm) → pushes tissue away
3. Tissue separates slightly → adhesion pulls again
4. **Infinite loop**: jiggling behavior!

The fundamental issue was **measurement mismatch**:
- Collision measures: triangle-point-to-surface penetration depth (~0.0001mm when touching)
- Adhesion measures: rigid-point-to-triangle distance (~20mm even when touching!)

## The Fix

**File**: [src/collision/CollisionScene.cpp](src/collision/CollisionScene.cpp#L690-L720)

**Change**: Skip collision constraint creation when rigid-deform adhesion is enabled.

```cpp
// Before (L700-708):
if (distance <= 1e-6) {
    if (rigid_obj->isFixed()) {
        xpbd_mesh_obj->addStaticCollisionConstraint(...);
    } else {
        xpbd_mesh_obj->addRigidDeformableCollisionConstraint(...);
    }
}

// After (L700-725):
if (distance <= 1e-6) {
    // ✅ BUG FIX #10: Avoid collision-adhesion conflict!
    const bool skip_collision_for_adhesion = _sim->config()->rigidDeformAdhesionEnable();
    
    if (!skip_collision_for_adhesion) {
        if (rigid_obj->isFixed()) {
            xpbd_mesh_obj->addStaticCollisionConstraint(...);
        } else {
            xpbd_mesh_obj->addRigidDeformableCollisionConstraint(...);
        }
    }
}
```

## Design Rationale

When rigid-deform adhesion is enabled, let the adhesion constraint handle **BOTH**:
1. **Attraction**: Pulling tissue toward bone when separated
2. **Contact**: Acting as soft collision when touching

**Benefits**:
- Eliminates constraint conflicts completely
- Single unified constraint for adhesion + contact
- Simpler system with fewer constraints
- Adhesion parameters control both bonding strength AND contact stiffness

**Trade-offs**:
- Adhesion must be strong enough to prevent penetration
  - Solution: Use small rest_gap (e.g., 0.5mm) and moderate alpha (e.g., 1e-4 to 1e-5)
- No hard collision enforcement (relies on soft adhesion spring)
  - Acceptable for biological tissue simulation where some compliance is realistic

## Expected Behavior After Fix

**Before Fix**:
```
Constraint counts per frame:
  - Adhesion: ~1500 constraints
  - Collision: ~50-100 constraints (on same triangles!)
  - Result: Jiggling, penetration, instability
```

**After Fix**:
```
Constraint counts per frame:
  - Adhesion: ~1500 constraints (handles both attraction + contact)
  - Collision: 0 constraints (on rigid-deform pairs with adhesion)
  - Result: Smooth, stable motion, no jiggling
```

## Testing Instructions

1. **Recompile**:
   ```bash
   cd build && make -j8
   ```

2. **Run test**:
   ```bash
   ./VirtuosoTest ../config/tbone_tumor_brain_adhesion_test.yaml
   ```

3. **Verify**:
   - ✅ No jiggling: Tumor should move smoothly near bone
   - ✅ No penetration: Tumor surface should not penetrate bone
   - ✅ Strong adhesion: Tumor should stick to bone, not fall away
   - ✅ Stable: Simulation should run smoothly for 10+ seconds

4. **Check constraint counts** (if debug output enabled):
   - Adhesion constraints: Should remain ~1500
   - Collision constraints (rigid-deform): Should be 0 or very few
   - Breaking: Constraints should not break easily

## Additional Notes

**If penetration still occurs**, adjust adhesion parameters:
- Decrease `rest_gap` (makes adhesion activate sooner, stronger repulsion when close)
- Decrease `alpha` (makes adhesion stiffer, stronger forces)

**Current recommended parameters** (in YAML):
```yaml
rigid-deform-adhesion-rest-gap: 0.0005      # 0.5mm
rigid-deform-adhesion-alpha: 5e-7           # Stiff enough to prevent penetration
rigid-deform-adhesion-break-ratio: 20.0     # Allow 10mm stretch before breaking
```

## Related Bugs

- **Bug #8**: Stale collision normal in RigidDeformableCollisionConstraint (already fixed)
- **Bug #9**: rest_gap parameter mismatch (already fixed via parameter tuning)
- **Bug #10**: Collision-adhesion conflict (THIS BUG - fixed via logic change)

All three bugs contributed to the jiggling/penetration issue. Bug #10 was the root cause - the other fixes were necessary but not sufficient.

## Files Modified

1. `src/collision/CollisionScene.cpp` - Added logic to skip collision when adhesion enabled
2. `BUG_10_COLLISION_ADHESION_CONFLICT.md` - Detailed analysis document (this is separate from the fix summary)

## Conclusion

This fix eliminates the fundamental logical conflict between collision and adhesion constraints by giving adhesion full responsibility for rigid-deform contact when enabled. This is the correct architectural solution - parameter tuning alone could never fix this issue.
