#!/bin/bash

# Development script for quick iteration on the refactored N-body simulation

set -e  # Exit on error

echo "==============================================="
echo "N-Body Simulation - Development Build & Test"
echo "==============================================="

# Function to build and run
build_and_run() {
    local target=$1
    local description=$2
    
    echo ""
    echo "Building $description..."
    cmake --build build -j --target $target
    
    if [ $? -eq 0 ]; then
        echo "✅ Build successful for $description"
        echo "🚀 Launching $description..."
        echo "(Close window to continue)"
        ./build/$target
    else
        echo "❌ Build failed for $description"
        exit 1
    fi
}

# Parse command line arguments
case "${1:-both}" in
    "original")
        build_and_run "raylib_nbody" "Original Version"
        ;;
    "refactored")
        build_and_run "raylib_nbody_refactored" "Refactored Version"
        ;;
    "both")
        echo "Building both versions..."
        cmake --build build -j
        
        if [ $? -eq 0 ]; then
            echo "✅ Both builds successful"
            echo ""
            echo "Choose version to run:"
            echo "1) Original"
            echo "2) Refactored"
            echo "3) Both (sequential)"
            read -p "Enter choice (1-3): " choice
            
            case $choice in
                1)
                    build_and_run "raylib_nbody" "Original Version"
                    ;;
                2)
                    build_and_run "raylib_nbody_refactored" "Refactored Version"
                    ;;
                3)
                    echo "Running both versions sequentially..."
                    build_and_run "raylib_nbody" "Original Version"
                    build_and_run "raylib_nbody_refactored" "Refactored Version"
                    ;;
                *)
                    echo "Invalid choice"
                    exit 1
                    ;;
            esac
        else
            echo "❌ Build failed"
            exit 1
        fi
        ;;
    *)
        echo "Usage: $0 [original|refactored|both]"
        exit 1
        ;;
esac

echo ""
echo "==============================================="
echo "Development session complete!"
echo "==============================================="