## Stretch Test Script
## This script demonstrates nerve constraint contribution by running two tests:
## 1. With nerve constraints enabled
## 2. With nerve constraints disabled

#!/bin/bash

echo "=================== Nerve Stretch Constraint Demonstration ==================="
echo "This test will demonstrate the difference in tissue behavior with/without nerve constraints"
echo

# Check if mesh exists
MESH_FILE="../resource/cube/cube8_nerve.msh"
if [ ! -f "$MESH_FILE" ]; then
    echo "Warning: Mesh file $MESH_FILE not found"
    echo "Using default cube mesh for demonstration"
fi

# Set environment variables
export NERVE_MSH="$MESH_FILE"
export NERVE_PHYS="nerve_edge"

CONFIG_FILE="./config/demos/nerve_test/stretch_test.yaml"
EXECUTABLE="./build/Test"

# Test 1: With nerve constraints
echo "=================== TEST 1: WITH Nerve Constraints ==================="
echo "Expected behavior: Tissue stretches but nerve edges maintain their length"
echo "Key observations:"
echo "  - Nerve constraint messages during setup"
echo "  - Edge length preservation in debug output"
echo "  - Limited overall deformation due to constraint"
echo

export NERVE_ENABLE=1
echo "Running: NERVE_ENABLE=1 $EXECUTABLE $CONFIG_FILE"
echo "Press Ctrl+C when you've observed the behavior..."
$EXECUTABLE "$CONFIG_FILE"

echo
echo "=================== TEST 2: WITHOUT Nerve Constraints ==================="
echo "Expected behavior: Tissue stretches freely, nerve edges deform proportionally"
echo "Key observations:"
echo "  - No nerve constraint setup messages"
echo "  - Free deformation of all tissue elements"
echo "  - Greater overall deformation possible"
echo

export NERVE_ENABLE=0
echo "Running: NERVE_ENABLE=0 $EXECUTABLE $CONFIG_FILE"
echo "Press Ctrl+C when you've observed the behavior..."
$EXECUTABLE "$CONFIG_FILE"

echo
echo "=================== Comparison Summary ==================="
echo "Key differences to look for:"
echo "1. Constraint Setup:"
echo "   - WITH: '[nerve] addNerveStretchConstraint: ok=X'"
echo "   - WITHOUT: '[nerve] NERVE_ENABLE=0/false; nerve constraints DISABLED'"
echo
echo "2. Edge Length Monitoring:"
echo "   - WITH: '[pre]/[post] edge(...) len = X.XXX (rest = Y.YYY)' with minimal difference"
echo "   - WITHOUT: No edge monitoring output"
echo
echo "3. Visual Deformation:"
echo "   - WITH: More constrained, stiffer behavior"
echo "   - WITHOUT: Freer deformation"
echo