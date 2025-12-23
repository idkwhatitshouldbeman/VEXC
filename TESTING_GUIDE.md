# VEX Robot Testing Guide

This guide covers how to test your VEX robot code both before and after uploading to hardware.

## Pre-Upload Testing

### 1. Compilation Test
```bash
cd /home/a_a_k/Downloads/VEXC
pros make clean
pros make
```
✅ **Expected**: All files compile with no errors (warnings are OK)

### 2. Logic Tests (No Hardware Required)
```bash
cd tests
clang++ -std=c++17 test_logic.cpp -o test_logic && ./test_logic
```
✅ **Expected**: All 10 tests pass

## Hardware Testing

### Prerequisites
1. VEX V5 Brain connected via USB
2. V5 Controller paired
3. SD Card inserted (for calibration and logging)
4. All motors and sensors connected according to port configuration

### Step 1: Upload Code
```bash
cd /home/a_a_k/Downloads/VEXC
pros upload
```

Or build and upload in one command:
```bash
pros mu
```

### Step 2: Initial Setup

1. **Power on the robot** - Brain should boot with your code
2. **Check terminal output** - Connect via PROS Terminal or wireless
3. **Verify initialization**:
   - Brain screen should show "Robot Ready!"
   - Terminal should show "IMU calibrated!"
   - No error messages

### Step 3: Calibration Mode

**Enter Calibration:**
- Hold **UP + X** buttons on controller during startup
- Robot will enter calibration mode

**Calibration Sequence:**
1. Follow on-screen prompts
2. Robot will move to measure tracking wheel positions
3. PID values will be auto-tuned
4. All values saved to SD card (`/usd/calibration.json`)

✅ **Success**: Calibration file created on SD card

### Step 4: Driver Control Testing

**Basic Drive Test:**
1. Move left stick Y → Left motors should move
2. Move right stick Y → Right motors should move
3. Both sticks forward → Robot drives forward
4. Both sticks backward → Robot drives backward
5. Opposite sticks → Robot turns in place

**Button Tests:**
- **A**: Toggle slow drive (should reduce max speed)
- **R1**: Toggle intake forward
- **R2**: Toggle intake reverse
- **L1**: Release mogo clamp
- **L2**: Clamp mogo
- **UP**: High score wait position
- **DOWN**: High score down position
- **LEFT**: High score scoring position
- **RIGHT**: High score capture position
- **X**: Toggle intake pneumatic
- **Y**: Doinker on
- **B**: Doinker off
- **LEFT + A**: MID_MODE selection

**Odometry Test:**
- Drive robot in a square pattern
- Check brain screen for position updates
- Position should track movement accurately

### Step 5: Autonomous Testing

**Select Autonomous:**
- During competition initialization, use controller buttons:
  - **UP + LEFT**: Red Left
  - **UP + RIGHT**: Red Right
  - **DOWN + LEFT**: Blue Left
  - **DOWN + RIGHT**: Blue Right
  - **A**: Skills
  - **B**: Test

**Test Autonomous:**
1. Put robot in autonomous mode
2. Robot should execute selected routine
3. Monitor terminal for debug output
4. Check that robot follows paths correctly

**Manual Autonomous Trigger (Non-Competition):**
- Hold **A + B** buttons during driver control
- Robot will run autonomous routine

### Step 6: Mechanism Testing

**Intake System:**
1. Press R1 → Intake should run forward
2. Press R1 again → Intake should stop
3. Press R2 → Intake should run reverse
4. Place ring in intake → Should detect and handle stall

**High Scoring:**
1. Press RIGHT → Should move to capture position
2. Press UP → Should move to wait position
3. Press LEFT → Should move to score position
4. Press DOWN → Should move to down position

**Pneumatics:**
1. Press L1 → Mogo clamp should release
2. Press L2 → Mogo clamp should engage
3. Press X → Intake pneumatic should toggle
4. Press Y → Doinker should activate
5. Press B → Doinker should deactivate

**Color Detection:**
1. Place colored ring in intake
2. System should detect color
3. Ejection should trigger for wrong color (if configured)

### Step 7: Path Following Test

**Create Test Path:**
1. Design a simple path in Jerry.io
2. Export as CSV to `paths/` folder
3. Create instruction file:
   ```
   INIT 0 0 0
   PATH test_path.csv
   WAIT_PATH
   END
   ```

**Run Path:**
1. Load instruction file in autonomous
2. Robot should follow path smoothly
3. Check odometry matches expected path

### Step 8: Telemetry Verification

**Brain Screen:**
- Should display:
  - X, Y position
  - Heading (degrees)
  - Battery percentage
  - Intake status
  - Mogo clamp status

**Controller Screen:**
- Should display:
  - Battery percentage
  - Position coordinates
  - Heading

**Terminal Output:**
- Should show status updates every second
- Debug messages if enabled

**SD Card Logging:**
- Check `/usd/match_log.csv` after running
- Should contain position data

### Step 9: Stress Testing

**Long Duration Test:**
1. Run driver control for 5+ minutes
2. Check for memory leaks or crashes
3. Monitor motor temperatures

**Rapid Input Test:**
1. Rapidly press buttons
2. System should handle all inputs correctly
3. No missed commands or conflicts

**Battery Test:**
1. Run until battery low
2. System should warn at 20% capacity
3. Robot should handle low battery gracefully

## Troubleshooting

### Robot Won't Initialize
- Check all motor/sensor connections
- Verify port assignments in `config.hpp`
- Check terminal for error messages

### Odometry Not Working
- Verify IMU is calibrated (wait for calibration)
- Check tracking wheel encoders are connected
- Verify encoders are reading (check terminal)

### Motors Not Moving
- Check motor ports match configuration
- Verify motor cartridges (blue/green)
- Check motor brake modes

### Autonomous Not Running
- Verify autonomous slot is selected
- Check instruction file syntax
- Verify path files exist

### Calibration Fails
- Ensure SD card is inserted
- Check SD card has space
- Verify file permissions

## Performance Benchmarks

**Expected Performance:**
- Odometry update rate: 100Hz (10ms delay)
- Driver control loop: 50Hz (20ms delay)
- Autonomous loop: 50Hz (20ms delay)
- Telemetry update: 10Hz (100ms delay)

**Position Accuracy:**
- Should track position within ±0.5 inches
- Heading accuracy: ±2 degrees

## Safety Checklist

Before competition:
- [ ] All code compiled successfully
- [ ] Calibration data loaded
- [ ] Driver controls tested
- [ ] Autonomous routines tested
- [ ] Battery fully charged
- [ ] SD card inserted
- [ ] All mechanisms functional
- [ ] Odometry verified
- [ ] Backup code on GitHub

## Next Steps

1. **Fine-tune PID values** - Adjust in calibration mode
2. **Create competition paths** - Design in Jerry.io
3. **Practice autonomous** - Test all routines
4. **Driver practice** - Get comfortable with controls
5. **Backup everything** - Save to GitHub regularly

## Quick Test Commands

```bash
# Build and upload
pros mu

# View terminal output
pros terminal

# Check SD card files
# (Use PROS file browser or terminal commands)

# Run logic tests
cd tests && clang++ -std=c++17 test_logic.cpp -o test_logic && ./test_logic
```

## Getting Help

- Check terminal output for error messages
- Review PROS documentation: https://pros.cs.purdue.edu/v5/
- Check GitHub issues
- Review code comments in source files

