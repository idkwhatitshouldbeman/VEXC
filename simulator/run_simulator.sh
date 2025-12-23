#!/bin/bash
# Quick start script for visual simulator

cd "$(dirname "$0")"

echo "Building visual simulator..."
clang++ -std=c++17 visual_simulator.cpp -o visual_simulator $(sdl2-config --cflags --libs) 2>&1

if [ $? -eq 0 ]; then
    echo "Build successful! Starting simulator..."
    echo ""
    ./visual_simulator
else
    echo "Build failed! Make sure SDL2 is installed:"
    echo "  Arch: sudo pacman -S sdl2"
    echo "  Ubuntu: sudo apt-get install libsdl2-dev"
    echo "  macOS: brew install sdl2"
    exit 1
fi

