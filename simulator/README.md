# Visual Simulator

A graphical simulator for testing your VEX robot code visually on your computer.

## Features

- ✅ **Visual Field Display** - See the 12x12 foot VEX field
- ✅ **Interactive Robot** - Click to set position, add waypoints
- ✅ **Real-time Movement** - Watch robot move with keyboard controls
- ✅ **Odometry Display** - See estimated position vs actual
- ✅ **Waypoint System** - Click to add waypoints, robot tracks them
- ✅ **Sensor Visualization** - See IMU, encoder readings

## Installation

### Arch Linux:
```bash
sudo pacman -S sdl2
```

### Ubuntu/Debian:
```bash
sudo apt-get install libsdl2-dev
```

### macOS:
```bash
brew install sdl2
```

## Compilation

```bash
cd simulator
clang++ -std=c++17 visual_simulator.cpp -o visual_simulator `sdl2-config --cflags --libs`
```

Or with g++:
```bash
g++ -std=c++17 visual_simulator.cpp -o visual_simulator `sdl2-config --cflags --libs`
```

## Running

```bash
./visual_simulator
```

## Controls

| Input | Action |
|-------|--------|
| **Left Click** | Add waypoint at mouse position |
| **Right Click** | Set robot position to mouse location |
| **Arrow Up** | Increase forward velocity |
| **Arrow Down** | Decrease/reverse velocity |
| **Arrow Left** | Turn left |
| **Arrow Right** | Turn right |
| **Space** | Stop robot (zero velocity) |
| **R** | Reset robot to center |
| **O** | Toggle odometry display |
| **S** | Toggle sensor display |
| **C** | Clear all waypoints |
| **ESC/Close** | Exit simulator |

## What You Can Test

1. **Robot Movement** - Use arrow keys to drive robot around
2. **Waypoint Following** - Add waypoints and see if robot can reach them
3. **Odometry Accuracy** - Compare actual position (blue) vs odometry (green)
4. **Path Planning** - Visualize paths before running on real robot
5. **Position Tracking** - See how position updates in real-time

## Display

- **Blue Rectangle** - Actual robot position and heading
- **Red Arrow** - Robot heading direction
- **Green Circle** - Odometry estimated position (if enabled)
- **Red Circles** - Waypoints (turn green when reached)
- **Gray Grid** - Field tiles (12" x 12")

## Limitations

- No physics simulation (robot moves perfectly)
- No collision detection
- Simplified odometry model
- No mechanism visualization (intake, scoring, etc.)

## Future Enhancements

- [ ] Physics simulation
- [ ] Obstacle avoidance
- [ ] Pure Pursuit path visualization
- [ ] Mechanism animations
- [ ] Multiple robot support
- [ ] Field element visualization (goals, barriers, etc.)
- [ ] Record and playback paths

