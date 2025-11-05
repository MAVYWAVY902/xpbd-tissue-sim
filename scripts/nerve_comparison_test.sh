#!/bin/bash

# Nerve Stretch Constraint Comparison Test Script
# This script runs the simulation twice - once with nerve constraints enabled,
# once with them disabled, to compare the behavior.

echo "=================== Nerve Stretch Constraint Comparison Test ==================="

# Set common environment variables
export NERVE_MSH="../resource/cube/cube8_nerve.msh"
export NERVE_PHYS="nerve_edge"

CONFIG_FILE="${1:-./config/example_config.yaml}"
BUILD_DIR="./build"
EXECUTABLE="$BUILD_DIR/Test"

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable not found at $EXECUTABLE"
    echo "Please build the project first: make -j$(nproc)"
    exit 1
fi

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found at $CONFIG_FILE"
    echo "Usage: $0 [config_file.yaml]"
    exit 1
fi

echo "Using config file: $CONFIG_FILE"
echo "Using nerve mesh: $NERVE_MSH"
echo

# Test 1: WITH nerve constraints
echo "=================== TEST 1: WITH Nerve Constraints ==================="
export NERVE_ENABLE=1
echo "Running with NERVE_ENABLE=1..."
echo "Expected: Nerve edges maintain length (error ~10^-6)"
echo "Command: NERVE_ENABLE=1 $EXECUTABLE $CONFIG_FILE"
echo
$EXECUTABLE "$CONFIG_FILE"
echo

# Test 2: WITHOUT nerve constraints  
echo "=================== TEST 2: WITHOUT Nerve Constraints ==================="
export NERVE_ENABLE=0
echo "Running with NERVE_ENABLE=0..."
echo "Expected: Nerve edges deform freely with tissue"
echo "Command: NERVE_ENABLE=0 $EXECUTABLE $CONFIG_FILE"
echo
$EXECUTABLE "$CONFIG_FILE"
echo

echo "=================== Comparison Complete ==================="
echo "Review the console output to compare:"
echo "  - WITH constraints: Look for '[nerve] addNerveStretchConstraint: ok=X'"
echo "  - WITHOUT constraints: Look for '[nerve] NERVE_ENABLE=0/false; nerve constraints DISABLED'"
echo "  - Monitor outputs: Compare [pre]/[post] edge length measurements"
echo
echo "Key differences to observe:"
echo "  1. Constraint addition messages"
echo "  2. Edge length preservation during simulation"
echo "  3. Visual deformation behavior (if graphics enabled)"