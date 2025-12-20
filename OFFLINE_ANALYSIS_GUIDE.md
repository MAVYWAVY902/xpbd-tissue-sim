# Offline Analysis System - Complete Guide

## System Overview

The offline analysis system consists of:
1. **C++ State Recorder** - Records simulation snapshots to binary file
2. **Python Analysis Tools** - Post-processes data to generate heatmaps, timelines, VTK sequences

## Current Implementation Status

### ✅ Fully Implemented
- SimulationStateRecorder class (C++ binary I/O)
- Configuration system (3 new parameters)
- Integration into Simulation class
- Python analysis framework
- Data structures for adhesions and deformations

### ⚠️ Partially Implemented
- Vertex position/velocity recording (WORKING)
- Adhesion state collection (INTERFACE EXISTS, but collection code NOT implemented)
- Deformation state collection (INTERFACE EXISTS, but collection code NOT implemented)

### ❌ Not Yet Working
- The simulation crashes with SIGSEGV when running
- No output files have been successfully generated yet

## Step-by-Step Testing Guide

### Important: Use the Right Test Program

**DO NOT USE**: `InitialDeformationTest` - This is a specialized test for elastic recovery, not suitable for state recording.

**RECOMMENDED**: Use `PushingTest` or `GraspingTest` - These are interactive simulations that properly call `Simulation::_timeStep()` where state recording happens.

### Step 1: Run PushingTest with State Recording

**Command**:
```bash
cd /home/yunxin/xpbd-tissue-sim/build
./PushingTest ../config/demos/simple_pushing/pushing_with_recording.yaml
```

**Command**:
```bash
cd /home/yunxin/xpbd-tissue-sim/build
./PushingTest ../config/demos/simple_pushing/pushing_with_recording.yaml
```

**What to Do**:
1. The visualization window will open
2. Use your mouse to push/interact with the cube
3. Let it run for at least 5-10 seconds to collect data
4. Close the window (this triggers the auto-save)

**Expected Console Output**:
```
[StateRecorder] Initialized with output folder: ../output/pushing_recording/
[StateRecorder] Recorded snapshot #1 at t=0.10s (frame 10) - 0 adhesions
[StateRecorder] Recorded snapshot #2 at t=0.20s (frame 20) - 0 adhesions
...
[StateRecorder] Auto-saving 50 snapshots...
[StateRecorder] Saved 50 snapshots to: ../output/pushing_recording/state_snapshots.bin
```

### Step 2: Verify Output Files

### Step 2: Verify Output Files

**Check files exist**:
```bash
ls -lh /home/yunxin/xpbd-tissue-sim/output/pushing_recording/
# Should show:
# state_snapshots.bin (size > 0)
# summary.txt
```

**Verify file contents**:
```bash
cat /home/yunxin/xpbd-tissue-sim/output/pushing_recording/summary.txt
```

### Step 3: Analyze the Data

### Step 3: Analyze the Data

Run the Python analysis tool:

```bash
cd /home/yunxin/xpbd-tissue-sim/scripts
python offline_analysis.py \
    --input ../output/pushing_recording/state_snapshots.bin \
    --output ../output/analysis/
```

**Expected Output**:
- `adhesion_breakage_timeline.png` - Timeline of breakage events
- `adhesion_strength_heatmap.png` - Heatmap of adhesion strength
- `vertex_velocity_heatmap.png` - Vertex velocity over time
- `vtk_sequence/*.vtu` - VTK files for ParaView

### Step 4: Visualize in ParaView (Optional)

```bash
# Open VTK sequence
paraview output/analysis/vtk_sequence/frame_0000.vtu
# Use "Time" slider to animate
```

## Key Code Locations

### C++ State Recording
- **Header**: `include/simulation/SimulationStateRecorder.hpp`
- **Implementation**: `src/simulation/SimulationStateRecorder.cpp`
- **Integration**: `src/simulation/Simulation.cpp` lines ~1220 and ~3140-3210
- **Config**: `include/config/simulation/SimulationConfig.hpp`

### Python Analysis
- **Main Tool**: `scripts/offline_analysis.py`
- **Test Scripts**: 
  - `scripts/test_offline_system.py`
  - `scripts/demo_offline_analysis.py`

### Configuration Files
- **Working Config**: `config/demos/nerve_test/ellipsoid_adhesion_test.yaml`
- **Test Config**: `config/demos/offline_recording_test.yaml` (has mesh file issues)

## Data Collection Details

### Currently Collected (WORKING)
```cpp
// In Simulation::_timeStep() around line 3147
for (const auto& obj : xpbd_objs) {
    // Collect vertex positions
    snapshot.vertex_positions.push_back(vertices.col(i));
    
    // Collect velocities
    snapshot.vertex_velocities.push_back(obj->vertexVelocity(i));
}
```

### NOT Yet Collected (Need Implementation)

**Adhesion States** - Need to add:
```cpp
// TODO: Add this to Simulation::_timeStep()
// Iterate through adhesion constraints
for (const auto& constraint : _adhesion_constraints) {
    SimulationStateRecorder::AdhesionState state;
    state.nerve_vertex_id = constraint.getNerveVertexId();
    state.is_broken = constraint.isBroken();
    // ... fill other fields
    snapshot.adhesion_states.push_back(state);
}
```

**Deformation States** - Need to add:
```cpp
// TODO: Compute strain/stress for each element
for (const auto& element : mesh->elements()) {
    SimulationStateRecorder::DeformationState state;
    state.element_id = element.id;
    state.volumetric_strain = computeVolumetricStrain(element);
    // ... fill other fields
    snapshot.deformation_states.push_back(state);
}
```

## Troubleshooting

### Problem: No output files created
- **Cause**: Simulation didn't complete, destructor not called
- **Solution**: Ensure simulation completes normally, or add explicit save call before exit

### Problem: File size is 0 or very small
- **Cause**: No snapshots recorded, or snapshots empty
- **Solution**: Check console output, verify `shouldRecord()` returns true

### Problem: Python script fails to load file
- **Cause**: Binary format mismatch
- **Solution**: Check C++ struct packing, ensure double=8 bytes on both sides

### Problem: Crash in state recorder
- **Cause**: Accessing invalid pointers in vertex collection loop
- **Solution**: Add null checks:
  ```cpp
  if (!mesh || mesh->numVertices() == 0) continue;
  ```

## Next Steps (Priority Order)

1. **[URGENT]** Fix the SIGSEGV crash
   - Debug with gdb
   - Add safety checks in vertex collection loop
   - Verify all pointers are valid

2. **[MEDIUM]** Implement adhesion state collection
   - Add accessors to constraint classes
   - Iterate through constraints in _timeStep()

3. **[LOW]** Implement deformation state collection
   - Add strain/stress computation
   - Store per-element data

4. **[OPTIONAL]** Optimize performance
   - Reduce snapshot frequency
   - Use compression
   - Stream to disk instead of storing in memory

## Contact

For issues or questions, check:
- Console output for error messages
- Log file: `build/InitialDeformationTest.log`
- Add debug prints in `SimulationStateRecorder.cpp`
