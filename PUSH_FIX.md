# Root Cause of Performance Issue

## Problem
PushingSimulation was using attachment constraints, which causes:
1. Every frame adds/removes hundreds of constraints
2. Every frame rebuilds VBD attachment lookup table
3. VBD solver iterates over large number of constraints
4. **EXTREMELY SLOW** - attachment constraints designed for grasping (few, persistent), not pushing (many, transient)

## Solution
**Revert to direct position manipulation** but fix VBD to handle it correctly:

### In PushingSimulation._applyPushingForces():
- Remove attachment constraint logic
- Use simple direct position displacement:
  ```cpp
  Real push_mag = penetration * 0.5;
  Vec3r new_pos = vertex_pos + push_direction * push_mag;
  mesh->setVertex(v, new_pos);
  ```

### In VBD solver:
The issue is that when positions are modified BEFORE update(), `_previous_vertices` captures the MODIFIED position, so inertial force becomes zero.

**Fix**: In VBD, use `_previous_vertices` (from PREVIOUS frame) as inertial reference, not `_inertial_vertices`.

Change line ~1790 in XPBDMeshObject.cpp from:
```cpp
const Vec3r x_inertia = _inertial_vertices.col(vid);
force = mass / (dt * dt) * (x_inertia - x_current);
```

To:
```cpp  
const Vec3r x_prev_frame = _previous_vertices.col(vid);
force = mass / (dt * dt) * (x_prev_frame - x_current);
```

This makes VBD resistant to external position manipulation.
