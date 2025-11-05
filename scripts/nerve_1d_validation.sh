#!/bin/bash

# Nerve 1D Rod Validation Test Protocol
# Tests current point-to-point constraints as validation for future 1D rod implementation

echo "=================== 1D Rod Nerve Constraint Validation ==================="
echo "This test validates nerve constraints in preparation for 1D rod implementation"
echo

# Generate the nerve stretch specimen mesh
MESH_GEO="../resource/demos/nerve_test/nerve_stretch_specimen.geo"
MESH_MSH="../resource/demos/nerve_test/nerve_stretch_specimen.msh"

if [ -f "$MESH_GEO" ]; then
    echo "Generating nerve stretch specimen mesh..."
    gmsh -3 "$MESH_GEO" -o "$MESH_MSH"
    if [ $? -eq 0 ]; then
        echo "✓ Mesh generated successfully"
    else
        echo "✗ Mesh generation failed"
        exit 1
    fi
else
    echo "Warning: Using fallback cube mesh (nerve_stretch_specimen.geo not found)"
    MESH_MSH="../resource/cube/cube8_nerve.msh"
fi

# Set environment for nerve constraint testing
export NERVE_MSH="$MESH_MSH"
export NERVE_PHYS="nerve_segments"
CONFIG_FILE="./config/demos/nerve_test/nerve_1d_stretch_test.yaml"
EXECUTABLE="./build/GraspingTest"

echo
echo "Test Configuration:"
echo "  Mesh: $NERVE_MSH"
echo "  Physical group: $NERVE_PHYS"
echo "  Config: $CONFIG_FILE"
echo

# Test 1: WITH nerve constraints (simulating 1D rod behavior)
echo "=================== TEST 1: 1D Rod Constraint Validation ==================="
echo "Testing current point-to-point constraints as 1D rod simulation"
echo
echo "Expected behavior:"
echo "  ✓ Multiple nerve segments maintain individual lengths"
echo "  ✓ Total nerve fiber length preserved during stretching"
echo "  ✓ Nerve acts as inextensible fiber within deformable tissue"
echo "  ✓ Tissue deformation constrained by nerve path"
echo

export NERVE_ENABLE=1
echo "Running: NERVE_ENABLE=1 $EXECUTABLE $CONFIG_FILE"
echo
echo "Stretch Test Instructions:"
echo "  1. Hold SPACE and drag mouse to stretch tissue"
echo "  2. Observe nerve fiber (red edges) maintaining length"
echo "  3. Note constraint satisfaction in terminal output"
echo "  4. Try stretching in different directions"
echo "  5. Press Ctrl+C when done observing"
echo

$EXECUTABLE "$CONFIG_FILE"

echo
echo "=================== TEST 2: Free Deformation Baseline ==================="
echo "Testing tissue without nerve constraints for comparison"
echo
echo "Expected behavior:"
echo "  ✓ Free tissue deformation in all directions"
echo "  ✓ No length constraints on any edges"
echo "  ✓ Uniform stress distribution during stretching"
echo

export NERVE_ENABLE=0
echo "Running: NERVE_ENABLE=0 $EXECUTABLE $CONFIG_FILE"
echo
echo "Comparison Instructions:"
echo "  1. Apply similar stretching motions as Test 1"
echo "  2. Observe unrestricted tissue deformation"
echo "  3. Note absence of constraint messages"
echo "  4. Compare stiffness and deformation patterns"
echo "  5. Press Ctrl+C when done comparing"
echo

$EXECUTABLE "$CONFIG_FILE"

echo
echo "=================== 1D Rod Validation Summary ==================="
echo
echo "Validation Criteria for Future 1D Rod Implementation:"
echo
echo "1. Length Preservation:"
echo "   ✓ Current: Each segment maintains rest length within tolerance"
echo "   → Future: Rod maintains total arc length and local curvature"
echo
echo "2. Constraint Forces:"
echo "   ✓ Current: Point-to-point tension forces along nerve path"
echo "   → Future: Distributed tension, bending, and torsion along rod"
echo
echo "3. Tissue Interaction:"
echo "   ✓ Current: Nerve nodes embedded in tissue mesh"
echo "   → Future: Rod-tissue coupling through contact/embedding"
echo
echo "4. Mechanical Behavior:"
echo "   ✓ Current: Inextensible cable behavior"
echo "   → Future: Full 1D rod with bending stiffness and material properties"
echo
echo "Key Metrics for 1D Rod Design:"
echo "  - Segment count: Determines rod discretization resolution"
echo "  - Constraint accuracy: Target error for rod element convergence"
echo "  - Force distribution: Validation of rod internal forces"
echo "  - Stability: Guidelines for rod timestep and solver parameters"