# VEXC - VEX V5 Robot Control System

A comprehensive, professional-grade VEX V5 robotics control system using PROS in C++.

## Features

- **Automatic Calibration** - Auto-calibrates tracking wheel positions, wheel diameters, and PID values
- **Pure Pursuit Path Following** - Smooth path following with Jerry.io CSV integration
- **Custom Instruction System** - G-code-like instructions for autonomous routines
- **XABY Button Mechanism Control** - Intuitive driver controls
- **Odometry System** - 100Hz position tracking using tracking wheels + IMU
- **Telemetry & Logging** - Wireless terminal output, brain screen display, SD card logging

## Project Structure

```
VEXC/
├── include/                    # Header files
│   ├── api.h                   # PROS API header
│   ├── main.h                  # Main header with includes
│   ├── config.hpp              # Hardware configuration
│   ├── utils.hpp               # Utility functions
│   ├── pid.hpp                 # PID controller
│   ├── odometry.hpp            # Position tracking
│   ├── pure_pursuit.hpp        # Path following
│   ├── mechanisms.hpp          # Intake, scoring, pneumatics
│   ├── driver_control.hpp      # Driver controls
│   ├── autonomous.hpp          # Autonomous routines
│   ├── calibration.hpp         # Calibration system
│   └── telemetry.hpp           # Logging and display
│
├── src/                        # Source files
│   ├── main.cpp                # Entry point
│   ├── config.cpp              # Configuration implementation
│   ├── utils.cpp               # Utility implementations
│   ├── pid.cpp                 # PID controller
│   ├── odometry.cpp            # Odometry system
│   ├── pure_pursuit.cpp        # Pure pursuit
│   ├── mechanisms.cpp          # Mechanism control
│   ├── driver_control.cpp      # Driver control
│   ├── autonomous.cpp          # Autonomous routines
│   ├── calibration.cpp         # Calibration system
│   └── telemetry.cpp           # Telemetry
│
├── paths/                      # Jerry.io path files (CSV)
│   └── example_path.csv
│
├── instructions/               # Custom autonomous instructions
│   └── example_auto.txt
│
├── calibration_data/           # Saved calibration (SD card)
│
├── Makefile                    # Build configuration
├── project.pros                # PROS project file
└── README.md                   # This file
```

## Hardware Configuration

Based on port assignments from reference repository:

### Motor Ports
| Motor | Port | Notes |
|-------|------|-------|
| Left Motor A | 2 | Blue cartridge, reversed |
| Left Motor B | 3 | Blue cartridge, reversed |
| Left Motor C | 4 | Blue cartridge, reversed |
| Right Motor A | 5 | Blue cartridge |
| Right Motor B | 12 | Blue cartridge |
| Right Motor C | 17 | Blue cartridge |
| High Scoring | 20 | Green cartridge |
| Intake Lower | 21 | Green cartridge |
| Intake Upper | 13 | Green cartridge |

### Sensor Ports
| Sensor | Port | Notes |
|--------|------|-------|
| IMU | 9 | Inertial sensor |
| Vertical Encoder | 7 | Forward tracking |
| Horizontal Encoder | 6 | Sideways tracking |
| High Scoring Rotation | 19 | Position feedback |
| Color Sensor | 15 | Optical sensor |

### ADI (3-Wire) Ports
| Device | Port | Notes |
|--------|------|-------|
| Mogo Clamp | F | Mobile goal clamp |
| Ejection | A | Ring ejection |
| Doinker | H | Doinker mechanism |
| Intake Pneumatic | D | Intake control |

## Getting Started

### Prerequisites

1. Install PROS CLI: https://pros.cs.purdue.edu/v5/getting-started/
2. Install VEX extension in your IDE

### Building

```bash
# Navigate to project directory
cd VEXC

# Build the project
pros make

# Upload to robot
pros upload

# Build and upload
pros mu
```

### Calibration Mode

Hold **UP + X** buttons on controller during startup to enter calibration mode.

The calibration sequence will:
1. Calibrate tracking wheel positions
2. Verify wheel diameters
3. Measure track width
4. Characterize motion
5. Auto-tune PID controllers
6. Save all values to SD card

### Creating Autonomous Routines

#### Method 1: Instruction Files

Create a text file in the `instructions/` folder:

```
# My Autonomous
INIT 0 0 180

HIGH_SCORE WAIT
INTAKE COLLECT

PATH my_path.csv
WAIT_PATH

MOGO on
WAIT 100

SCORE TOP_BACK

END
```

#### Method 2: Jerry.io Paths

1. Design paths visually in Jerry.io (https://jerry.io)
2. Export as CSV to `paths/` folder
3. Reference in instruction file with `PATH filename.csv`

### Driver Controls

| Button | Function |
|--------|----------|
| Left Stick Y | Left drivetrain |
| Right Stick Y | Right drivetrain |
| A | Toggle slow drive |
| R1 | Toggle intake forward |
| R2 | Toggle intake reverse |
| L1 | Release mogo |
| L2 | Clamp mogo |
| UP | High score wait position |
| DOWN | High score down position |
| LEFT | High score scoring position |
| RIGHT | High score capture position |
| X | Toggle intake pneumatic |
| Y | Doinker on |
| B | Doinker off |

### Autonomous Selection

During competition initialization:
- **UP + LEFT**: Red Left
- **UP + RIGHT**: Red Right
- **DOWN + LEFT**: Blue Left
- **DOWN + RIGHT**: Blue Right
- **A**: Skills
- **B**: Test

## Thread Architecture

```
Thread 1 (HIGHEST): Odometry Update Loop - 100Hz
Thread 2 (HIGH): Driver Control - 50Hz
Thread 3 (MEDIUM): Autonomous Controller - 50Hz
Thread 4 (LOW): Telemetry & Monitoring - 10Hz
```

## Resources

- PROS Documentation: https://pros.cs.purdue.edu/v5/
- Jerry.io Path Designer: https://jerry.io
- Reference Repository: https://github.com/arvindkandhare/dt_working

## License

This project is for VEX Robotics competition use.
