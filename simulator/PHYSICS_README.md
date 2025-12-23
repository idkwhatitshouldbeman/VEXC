# Physics Simulator with PID Testing

A realistic physics-based simulator for testing PID controllers without hardware.

## Features

✅ **Realistic Physics**
- Mass, inertia, friction
- Motor torque limits
- Velocity and acceleration
- Realistic robot dynamics

✅ **PID Controller Testing**
- Test linear PID (driving to point)
- Test angular PID (heading correction)
- Test turn PID (point turns)
- Real-time PID tuning with keyboard

✅ **Visual Feedback**
- Robot position and heading
- Target position
- Trajectory path
- Force vectors (optional)
- PID term visualization

✅ **Multiple Control Modes**
- Manual (direct control)
- Drive-to (PID drives to clicked point)
- Turn-to (PID turns to heading)
- Follow-path (PID follows waypoints)

## Quick Start

```bash
cd simulator
clang++ -std=c++17 physics_simulator.cpp -o physics_simulator $(sdl2-config --cflags --libs)
./physics_simulator
```

## Controls

### Mode Selection
| Key | Mode | Description |
|-----|------|-------------|
| **M** | Manual | Direct keyboard control |
| **D** | Drive-to | Click to set target, PID drives there |
| **T** | Turn-to | PID turns to target heading |
| **F** | Follow-path | PID follows waypoint path |

### Movement (Manual Mode)
| Key | Action |
|-----|--------|
| **↑** | Increase forward force |
| **↓** | Decrease/reverse force |
| **←** | Turn left |
| **→** | Turn right |
| **Space** | Stop (zero forces) |

### PID Tuning (Real-time)
| Key | Action |
|-----|--------|
| **1** | Decrease Linear KP |
| **2** | Increase Linear KP |
| **3** | Decrease Linear KI |
| **4** | Increase Linear KI |
| **5** | Decrease Linear KD |
| **6** | Increase Linear KD |

### Display Toggles
| Key | Toggle |
|-----|--------|
| **P** | PID info display |
| **W** | Force vectors |
| **G** | Trajectory path |
| **O** | Odometry display |

### Other
| Key | Action |
|-----|--------|
| **Left Click** | Add waypoint / Set target |
| **Right Click** | Reset robot position |
| **R** | Reset everything |
| **C** | Clear waypoints |

## Testing PID Controllers

### 1. Drive-to Test
1. Press **D** to enter drive-to mode
2. Click anywhere on field to set target
3. Watch robot use PID to drive to target
4. Adjust PID gains with **1-6** keys
5. Observe how different gains affect performance

### 2. Turn-to Test
1. Press **T** to enter turn-to mode
2. Use **↑/↓** to adjust target heading
3. Watch robot use turn PID to reach heading
4. Tune turn PID gains

### 3. Path Following Test
1. Click multiple points to create waypoints
2. Press **F** to start following
3. Robot uses PID to navigate between waypoints
4. Observe trajectory and smoothness

## Physics Parameters

The simulator uses realistic physics:
- **Robot Mass**: 10 kg
- **Wheel Base**: 12.5 inches
- **Wheel Radius**: 1.625 inches (3.25" diameter)
- **Max Motor Torque**: 2.1 N⋅m (V5 motor spec)
- **Friction**: 0.3 coefficient
- **Moment of Inertia**: 0.5 kg⋅m²

## PID Performance Indicators

Watch for:
- **Overshoot**: Robot goes past target (increase KD, decrease KP)
- **Oscillation**: Robot wobbles (decrease KP, increase KD)
- **Slow Response**: Takes too long (increase KP)
- **Steady-state Error**: Never reaches target (increase KI)
- **Settling Time**: How long to reach target

## Tips for Tuning

1. **Start with P only** (KI=0, KD=0)
   - Increase KP until oscillation starts
   - Then back off 20%

2. **Add D** to reduce oscillation
   - Increase KD until smooth
   - Too much KD causes slow response

3. **Add I** to eliminate steady-state error
   - Use small KI values (0.01-0.1)
   - Too much KI causes overshoot

4. **Test different scenarios**
   - Short distances
   - Long distances
   - Sharp turns
   - Smooth curves

## What You Can Test

✅ **PID Controller Performance**
- Response time
- Overshoot
- Settling time
- Steady-state error

✅ **Robot Dynamics**
- Acceleration limits
- Turning radius
- Friction effects
- Momentum

✅ **Path Following**
- Waypoint navigation
- Smooth trajectories
- Obstacle avoidance (manual)

✅ **Real-world Behavior**
- How physics affects PID
- Motor limitations
- Sensor delays (simulated)

## Limitations

- No collision detection
- Simplified friction model
- No sensor noise simulation
- No mechanism physics (intake, scoring, etc.)

## Next Steps

1. Tune PID values in simulator
2. Copy tuned values to `config.hpp`
3. Test on real robot
4. Fine-tune based on real-world performance

The physics simulator helps you understand how PID works and find good starting values before testing on hardware!

