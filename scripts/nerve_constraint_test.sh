#!/bin/bash

# Nerve Constraint Comparison Test Script
# This script runs the gravity-based hanging beam test with and without nerve constraints
# to demonstrate the differences in behavior

echo "=========================================="
echo "NERVE CONSTRAINT COMPARISON TEST"
echo "=========================================="
echo ""
echo "This test demonstrates how nerve constraints affect tissue deformation"
echo "under gravity by running two scenarios:"
echo "1. WITH nerve constraints (limited stretching)"
echo "2. WITHOUT nerve constraints (more stretching)"
echo ""

# Navigate to build directory
cd /home/yunxin/xpbd-tissue-sim/build

# Test 1: WITH nerve constraints
echo "=========================================="
echo "TEST 1: Running with NERVE CONSTRAINTS"
echo "=========================================="
echo "Expected behavior: Tissue hangs from ceiling but nerve constraints"
echo "prevent excessive stretching. Beam should maintain some structural integrity."
echo ""
echo "Starting simulation..."
export NERVE_ENABLE=1
export NERVE_MSH="../resource/cube/cube8_nerve.msh"
export NERVE_PHYS="nerve_edge"
./Test ../config/demos/nerve_test/hanging_gravity_test.yaml

echo ""
echo "=========================================="
echo "TEST 1 COMPLETE"
echo "=========================================="
echo "Please observe and note the final stretched length and shape."
echo "Press any key to continue to the second test..."
read -n 1 -s

# Test 2: WITHOUT nerve constraints
echo ""
echo "=========================================="
echo "TEST 2: Running WITHOUT nerve constraints"
echo "=========================================="
echo "Expected behavior: Tissue hangs from ceiling with more stretching"
echo "allowed. Beam should stretch more under gravity without nerve constraints."
echo ""
echo "Starting simulation..."
export NERVE_ENABLE=0
export NERVE_MSH="../resource/cube/cube8_nerve.msh"  
export NERVE_PHYS="nerve_edge"
./Test ../config/demos/nerve_test/hanging_gravity_test.yaml

echo ""
echo "=========================================="
echo "TEST 2 COMPLETE"
echo "=========================================="
echo ""
echo "COMPARISON SUMMARY:"
echo "==================="
echo "Test 1 (WITH nerve constraints):"
echo "  - Should show limited stretching"
echo "  - Nerve constraints prevent excessive deformation"
echo "  - Tissue maintains more structural integrity"
echo ""
echo "Test 2 (WITHOUT nerve constraints):"
echo "  - Should show more stretching under gravity"
echo "  - Only tissue elasticity resists deformation"
echo "  - Beam may stretch significantly more"
echo ""
echo "The difference demonstrates that nerve constraints provide"
echo "additional structural support beyond tissue elasticity."
echo ""
echo "TEST COMPLETE!"