# VEX Robot Simulator Setup Guide

Unfortunately, most VEX simulators are archived/not actively maintained, but here are your options:

## Option 1: PROS Simulator (Rust-based) ⚠️ Archived

**Status**: Archived but may still work

**Installation:**
```bash
# Install Rust first
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env

# Install pros-simulator
cargo install pros-simulator

# Run your compiled binary
pros-simulator ./bin/hot.package.bin
```

**Repository**: https://github.com/vexide/pros-simulator

## Option 2: V5-Sim-Engine (LemLib) ⚠️ Archived

**Status**: Archived, development stopped

**Repository**: https://github.com/LemLib/v5-sim-engine

**Note**: This project was incomplete and may not work with current PROS versions.

## Option 3: VirtualVEX (Team 254)

**Status**: Active, but requires VEXcode (not PROS)

**Website**: https://www.team254.com/vex/vex-simulator/

**Note**: This only works with VEXcode, not PROS C++ code.

## Option 4: Mock Testing Environment ✅ **WORKING NOW!**

I've created a mock PROS API that lets you test your code logic without hardware!

### Quick Start:
```bash
cd tests
clang++ -std=c++17 test_with_mock.cpp -o test_with_mock && ./test_with_mock
```

### What's Included:
- ✅ **mock_pros.hpp** - Full mock of PROS API (Motors, IMU, Sensors, Controller)
- ✅ **test_with_mock.cpp** - Example tests using the mock API
- ✅ **test_logic.cpp** - Mathematical/logical tests

### Benefits:
- ✅ No hardware needed
- ✅ Tests all mathematical/logical code
- ✅ Validates algorithms (PID, odometry, pure pursuit)
- ✅ Fast iteration
- ✅ Can run in CI/CD
- ✅ Can simulate robot movement over time

### Limitations:
- ❌ Can't test actual motor behavior
- ❌ Can't test real sensor readings
- ❌ No physics simulation
- ❌ No visual feedback

### How to Use:

1. **Test Mock API:**
   ```bash
   cd tests
   clang++ -std=c++17 test_with_mock.cpp -o test_with_mock && ./test_with_mock
   ```

2. **Test Logic:**
   ```bash
   cd tests
   clang++ -std=c++17 test_logic.cpp -o test_logic && ./test_logic
   ```

3. **Extend for Your Code:**
   - Include your source files
   - Use `#define TEST_MODE` to conditionally compile
   - Link against `mock_pros.hpp` instead of real PROS
   - Create test scenarios

## Option 5: Unit Testing Framework

We can create a more comprehensive unit testing framework that mocks PROS API calls.

