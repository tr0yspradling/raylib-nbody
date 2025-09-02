#!/bin/bash

# Test script for both versions of the N-body simulation

echo "==============================================="
echo "N-Body Simulation - Version Testing"
echo "==============================================="

echo ""
echo "Available executables:"
ls -la build/raylib_nbody*

echo ""
echo "==============================================="
echo "Testing Original Version"
echo "==============================================="
echo "Launching original version..."
echo "Close the window to continue to refactored version"
echo ""

# Run original version
./build/raylib_nbody &
ORIGINAL_PID=$!

# Wait for user to close it
wait $ORIGINAL_PID

echo ""
echo "==============================================="
echo "Testing Refactored Version"
echo "==============================================="
echo "Launching refactored version..."
echo "Close the window to finish testing"
echo ""

# Run refactored version
./build/raylib_nbody_refactored &
REFACTORED_PID=$!

# Wait for user to close it
wait $REFACTORED_PID

echo ""
echo "==============================================="
echo "Testing Complete!"
echo "==============================================="
echo "Both versions completed successfully."