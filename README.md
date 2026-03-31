# 🤖 GELLO UR5 Gravity Compensation — Custom Build

> **Active gravity compensation for a custom-redesigned GELLO UR5 leader arm with XM430-W350-T motors at joints 2 & 3.**

A standalone, plug-and-play Python script that makes your GELLO arm feel weightless by continuously counteracting gravity using real-time inverse dynamics.

---

## 📖 Table of Contents

1. [What Is Gravity Compensation?](#what-is-gravity-compensation)
2. [How It Works — The Theory](#how-it-works--the-theory)
3. [Hardware Configuration](#hardware-configuration)
4. [Software Architecture](#software-architecture)
5. [Step-by-Step Setup Tutorial](#step-by-step-setup-tutorial)
6. [Usage Guide](#usage-guide)
7. [Tuning Guide](#tuning-guide)
8. [Implementation Deep Dive](#implementation-deep-dive)
9. [Troubleshooting](#troubleshooting)
10. [Credits](#credits)

---

## What Is Gravity Compensation?

When you release a robot arm, gravity pulls it down. **Gravity compensation** applies precisely calculated torques to each joint motor to counteract gravity, making the arm feel **weightless** — it stays wherever you place it.

This is essential for teleoperation with GELLO because:
- The operator can move the leader arm effortlessly
- No springs or rubber bands needed (active vs passive compensation)
- The arm holds any pose without drifting

### Before vs After

| Without Gravity Comp | With Gravity Comp |
|---------------------|-------------------|
| Arm collapses under its own weight | Arm floats in any position |
| Needs springs/rubber bands | No mechanical counterbalance |
| Limited workspace | Full range of motion |
| Operator fights gravity | Effortless manipulation |

---

## How It Works — The Theory

### 1. Rigid Body Dynamics (Newton-Euler)

Every link in the robot arm has mass and is affected by gravity. The torque needed at each joint to hold the arm stationary is computed using the **Recursive Newton-Euler Algorithm (RNEA)**:

```
τ_gravity = RNEA(model, q, dq, 0)
```

Where:
- `model` — The robot's physical description (link lengths, masses, inertias from the URDF)
- `q` — Current joint positions (radians)
- `dq` — Current joint velocities (rad/s)  
- `0` — Zero acceleration (we want static equilibrium, not movement)

The result `τ_gravity` is a vector of torques (in Nm) needed at each joint.

### 2. Torque → Current Conversion

Dynamixel servos are controlled via **current commands** (mA), not torque directly. Each motor type has a torque constant:

```
current_mA = torque_Nm × torque_constant_mA_per_Nm
```

| Motor | Torque Constant | Max Current |
|-------|----------------|-------------|
| XL330-M288-T | 1158.73 mA/Nm | 1100 mA |
| XM430-W350-T | 561.80 mA/Nm | 2000 mA |

### 3. Control Loop (500 Hz)

The gravity changes as the arm moves, so compensation must be continuous:

```
┌──────────────────────────────────────────────────┐
│                  CONTROL LOOP                     │
│                                                   │
│   ┌─────────┐   ┌──────────┐   ┌──────────────┐ │
│   │  READ   │──▶│ COMPUTE  │──▶│   WRITE      │ │
│   │ Sensors │   │ Torques  │   │  Currents    │ │
│   └─────────┘   └──────────┘   └──────────────┘ │
│       │              │               │            │
│   positions     gravity +        motor            │
│   velocities    friction +       commands         │
│                 joint limits                      │
│                                                   │
│          ◄──── repeats at 500 Hz ────►           │
└──────────────────────────────────────────────────┘
```

---

## Hardware Configuration

### Motor Layout

This script is designed for a **custom GELLO UR5** where joints 2 and 3 have been upgraded:

```
Joint 1 (Shoulder Pan)  ─── XL330-M288-T   (original, ID: 1)
Joint 2 (Shoulder Lift) ─── XM430-W350-T   (UPGRADED, ID: 2) ⭐
Joint 3 (Elbow)         ─── XM430-W350-T   (UPGRADED, ID: 3) ⭐
Joint 4 (Wrist 1)       ─── XL330-M288-T   (original, ID: 4)
Joint 5 (Wrist 2)       ─── XL330-M288-T   (original, ID: 5)
Joint 6 (Wrist 3)       ─── XL330-M288-T   (original, ID: 6)
Joint 7 (Gripper)       ─── XL330-M077-T   (original, ID: 7)
```

### Why XM430-W350-T for Joints 2 & 3?

These joints bear the most load (supporting the weight of the entire arm below them). The XM430:
- **4.1 Nm stall torque** (vs 0.62 Nm for XL330)
- **Current control mode** for precise torque output
- Same bolt pattern, easy drop-in replacement

### Wiring

```
PC ──USB──▶ U2D2 ──TTL──▶ Motor 1 ──▶ Motor 2 ──▶ ... ──▶ Motor 7
                          (daisy-chain all 7 motors)
```

---

## Software Architecture

### File Structure

| File | Description |
| :--- | :--- |
| [gravity_compensation.py](file:///C:/Users/alame/.gemini/antigravity/scratch/gello-gravity-comp/gravity_compensation.py) | Main script — self-contained with all classes |
| [gello_gui.py](file:///C:/Users/alame/.gemini/antigravity/scratch/gello-gravity-comp/gello_gui.py) | **New!** Premium GUI dashboard for easy setup |
| [gello_ur5.urdf](file:///C:/Users/alame/.gemini/antigravity/scratch/gello-gravity-comp/gello_ur5.urdf) | Robot model with updated masses for XM430 motors |
| [README.md](file:///C:/Users/alame/.gemini/antigravity/scratch/gello-gravity-comp/README.md) | Full documentation with tuning guide |
| [validate.py](file:///C:/Users/alame/.gemini/antigravity/scratch/gello-gravity-comp/validate.py) | Quick validation script |

### Class Diagram

```
gravity_compensation.py
│
├── DynamixelInterface          ← Handles all servo communication
│   ├── connect()               - Opens serial port, starts read thread
│   ├── read_positions_and_velocities()  - Returns (rad, rad/s)
│   ├── write_currents(mA)      - Sends current commands via sync write
│   ├── write_torques(Nm)       - Converts Nm → mA, then writes
│   ├── set_operating_mode()    - Switches current/position control
│   ├── enable_torque()         - Enables motor output
│   ├── disable_torque()        - Disables motor output
│   └── close()                 - Clean shutdown
│
├── GravityCompensator          ← Computes and applies compensation 
│   ├── initialize()            - Full startup sequence
│   ├── _load_urdf()            - Loads robot model via Pinocchio
│   ├── _calibrate()            - Computes joint offsets
│   ├── compute_gravity_torque()     - RNEA inverse dynamics
│   ├── compute_friction_torque()    - Static friction dithering
│   ├── compute_joint_limit_torque() - Barrier repulsion
│   ├── control_step()          - One iteration of the loop
│   ├── run()                   - Main control loop
│   └── shutdown()              - Zero torques + close
│
└── main()                      ← CLI entry point with argparse
```

### Dependencies

| Package | Purpose | Install |
|---------|---------|---------|
| `dynamixel-sdk` | Communicate with Dynamixel servos | `pip install dynamixel-sdk` |
| `pin` (Pinocchio) | Rigid-body dynamics / inverse dynamics | `pip install pin` |
| `numpy` | Numerical computation | `pip install numpy` |

---

## Step-by-Step Setup Tutorial

### Step 1: Clone This Repository

```bash
git clone https://github.com/Alameen7git/gello-gravity-comp.git
cd gello-gravity-comp
```

### Step 2: Install Python Dependencies

```bash
pip install dynamixel-sdk pin numpy
```

Verify installation:
```bash
python -c "import dynamixel_sdk; print('dynamixel-sdk OK')"
python -c "import pinocchio; print('pinocchio OK')"
python -c "import numpy; print('numpy OK')"
```

### Step 3: Set Up Dynamixel Motor IDs

Each motor needs a unique ID (1-7). If not already set:

1. Install [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
2. Connect **one motor at a time** to the U2D2
3. Scan → Select motor → Change ID:
   - Base motor → ID 1
   - Shoulder lift → ID 2
   - Elbow → ID 3
   - Wrist 1 → ID 4
   - Wrist 2 → ID 5
   - Wrist 3 → ID 6
   - Gripper → ID 7

### Step 4: Find Your Serial Port

**Linux:**
```bash
ls /dev/serial/by-id/
# Example output: usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0
```

**Windows:**
- Open Device Manager → Ports (COM & LPT)
- Look for "USB Serial Port (COM3)" or similar

**macOS:**
```bash
ls /dev/cu.usbserial-*
```

### Step 5: Set Latency Timer (Linux only, recommended)

For optimal 500Hz performance:
```bash
# Find your ttyUSB device
ls /dev/ttyUSB*

# Set latency to 1ms (default is 16ms)
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### Step 6: Add User to dialout Group (Linux only)

```bash
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```

### Step 7: Position the Arm for Calibration

Place your GELLO arm in the **calibration position** before starting the script:

```
Standard GELLO UR5 Home Position:
  Joint 1 (Shoulder Pan):  0°      (straight forward)
  Joint 2 (Shoulder Lift): -90°    (pointing up)
  Joint 3 (Elbow):         +90°    (bent forward)
  Joint 4 (Wrist 1):       -90°    (pointing down)
  Joint 5 (Wrist 2):       -90°    (rotated)
  Joint 6 (Wrist 3):       0°      (neutral)
```

> ⚠️ **Important**: The arm must be in this exact position when you start the script, or calibration will fail.

### Step 8: Launch the GUI (Recommended)

For a user-friendly experience, use the new GUI setup:

```bash
python gello_gui.py
```
This wizard will guide you through port selection, motor verification, and calibration with visual feedback and a live dashboard.

### Step 9: Test with Dry Run (CLI Alternative)

If you prefer the command line:

```bash
# Linux
python gravity_compensation.py --dry-run --port /dev/ttyUSB0

# Windows
python gravity_compensation.py --dry-run --port COM3
```

You should see output like:
```
============================================================
GELLO UR5 Gravity Compensation — Initializing
============================================================

[1/4] Connecting to Dynamixel servos...
Connected to 7 Dynamixel servos on /dev/ttyUSB0

[2/4] Loading URDF for inverse dynamics...
  URDF: gello_ur5.urdf
  Model DOFs: 6

[3/4] Calibrating joint offsets...
  Joint offsets: ['0.123', '-1.456', '2.789', ...]
  Calibration error: ['0.0012', '0.0034', ...]

[4/4] Configuring servos...
DRY RUN mode — torque will NOT be enabled

[DRY RUN] Positions (deg): [   0.1,  -89.8,   90.2, ...]
[DRY RUN] Gravity τ (Nm):  [ 0.0000,  0.0234,  0.0156, ...]
```

### Step 10: Start Setup

Once dry-run looks good, start with **low gain** (0.2-0.3):

```bash
python gravity_compensation.py --port /dev/ttyUSB0 --gain 0.2
```

- Gently move the arm — it should resist gravity slightly
- If any joint moves the **wrong direction** (amplifies instead of resists), stop immediately and flip that joint's sign (see [Tuning Guide](#tuning-guide))

### Step 11: Normal Operation

```bash
python gravity_compensation.py --port /dev/ttyUSB0 --gain 0.6
```

The arm should now feel nearly weightless. Adjust gain between 0.5-0.8 for best results.

### Step 11: Stop the Script

Press `Ctrl+C`. The script will:
1. Zero all torque commands
2. Disable motor torque
3. Close the serial connection

---

## Usage Guide

### Command-Line Options

```bash
python gravity_compensation.py [OPTIONS]
```

| Flag | Short | Description | Default |
|------|-------|-------------|---------|
| `--port` | `-p` | Serial port path | `/dev/ttyUSB0` |
| `--gain` | `-g` | Gravity compensation gain (0.0-1.0) | `0.6` |
| `--freq` | `-f` | Control loop frequency (Hz) | `500` |
| `--dry-run` | `-d` | Read-only mode, no torque output | `False` |
| `--urdf` | `-u` | Path to URDF file | `gello_ur5.urdf` |
| `--friction` | | Friction compensation gain | `0.0` |

### Examples

```bash
# Basic operation
python gravity_compensation.py -p /dev/ttyUSB0 -g 0.6

# Safe testing
python gravity_compensation.py -p /dev/ttyUSB0 --dry-run

# With friction compensation
python gravity_compensation.py -p /dev/ttyUSB0 -g 0.6 --friction 0.15

# Lower control rate (if CPU is slow)
python gravity_compensation.py -p /dev/ttyUSB0 -g 0.6 -f 200

# Windows
python gravity_compensation.py -p COM3 -g 0.6
```

### Editing Default Configuration

All defaults are in the `DEFAULT_CONFIG` dict at the top of `gravity_compensation.py`:

```python
DEFAULT_CONFIG = {
    "port": "/dev/ttyUSB0",
    "baudrate": 57600,
    "motor_ids": [1, 2, 3, 4, 5, 6, 7],
    "servo_types": [
        "XL330_M288_T",   # Joint 1
        "XM430_W350_T",   # Joint 2 (UPGRADED)
        "XM430_W350_T",   # Joint 3 (UPGRADED)
        "XL330_M288_T",   # Joint 4
        "XL330_M288_T",   # Joint 5
        "XL330_M288_T",   # Joint 6
        "XL330_M077_T",   # Gripper
    ],
    "joint_signs": [1, 1, -1, 1, 1, 1, 1],
    "gravity_comp_gain": 0.6,
    # ... more options
}
```

---

## Tuning Guide

### Gravity Compensation Gain (`--gain`)

This is the most important parameter. It scales the computed gravity torques:

| Gain | Behavior | When to Use |
|------|----------|-------------|
| **0.0** | No compensation | Motor free / backdrivable |
| **0.1-0.3** | Arm sags slowly | Initial testing |
| **0.4-0.6** | Arm mostly holds | Normal tuning range |
| **0.7-0.8** | Arm holds firmly | Well-tuned system |
| **>0.9** | May oscillate! | Only if masses are very accurate |

**Start at 0.3 and increase by 0.1 increments.**

### Joint Signs

If a joint moves the **wrong direction** when gravity compensation is active (pushes down instead of holding up), its sign is wrong.

```python
# In DEFAULT_CONFIG:
"joint_signs": [1, 1, -1, 1, 1, 1, 1],
#                      ^^
# Flip -1 to 1 or 1 to -1 for the problematic joint
```

**How to test:** With `--gain 0.1`, gently push each joint. It should resist your push slightly. If it helps your push (amplifies motion), flip that joint's sign.

### Friction Compensation (`--friction`)

When joints feel "sticky" at low speeds:

| Gain | Effect |
|------|--------|
| **0.0** | Disabled (default) |
| **0.05-0.15** | Subtle, helps with stiction |
| **0.2-0.3** | Aggressive dithering |
| **>0.3** | May cause vibration |

### URDF Mass Tuning

If gravity compensation is **uneven** (some joints compensate well, others don't):

Edit `gello_ur5.urdf` and adjust `<mass>` values:

```xml
<!-- Joint 2 link — increase if this joint sags -->
<link name="link2">
  <inertial>
    <mass value="0.155"/>  <!-- Try increasing to 0.18 or 0.20 -->
  </inertial>
</link>
```

---

## Implementation Deep Dive

### 1. Dynamixel Communication

The `DynamixelInterface` class uses the Dynamixel SDK Protocol 2.0:

**Background Read Thread (1kHz):**
```
GroupSyncRead → reads position + velocity from ALL motors in one packet
  ├── Address 128: Present Velocity (4 bytes, signed 32-bit)
  └── Address 132: Present Position (4 bytes, signed 32-bit)

Raw ticks → radians: position_rad = ticks / 2048.0 × π
Raw units → rad/s:   velocity_rad_s = units × 0.229 × 2π / 60
```

**Sync Write (per control step):**
```
GroupSyncWrite → writes goal current to ALL motors in one packet
  └── Address 102: Goal Current (2 bytes, signed 16-bit, units: mA)
```

### 2. Calibration

When the script starts, it needs to know the absolute joint angles. Since Dynamixel encoders are relative (they wrap around), we do a search:

```python
for each joint i:
    for offset in range(-20π, +20π, step=π/2):
        joint_angle = sign[i] * (raw_position[i] - offset)
        error = |joint_angle - known_calibration_angle[i]|
        keep offset with minimum error
```

This finds the offset that makes the raw encoder reading match the known calibration position.

### 3. Gravity Compensation (Pinocchio RNEA)

The core physics uses Pinocchio's `rnea()` function:

```python
import pinocchio as pin

# RNEA: Recursive Newton-Euler Algorithm
# With zero acceleration, this gives us gravity + Coriolis torques
tau_gravity = pin.rnea(
    model,          # Robot model from URDF
    data,           # Pinocchio data container
    q,              # Current joint positions [6]
    dq,             # Current joint velocities [6]
    np.zeros(6),    # Zero acceleration = gravity only
)

# Scale by gain (0.0 to 1.0)
tau_gravity *= gravity_comp_gain
```

**What RNEA does internally:**
1. Forward pass: propagates velocities/accelerations from base to tip
2. Backward pass: computes forces/torques from tip back to base
3. Each link's contribution: `τ = m × g × r` (mass × gravity × distance to joint)

### 4. Friction Compensation

Servo gears have static friction (stiction). To overcome it, we apply alternating small torques when the joint is stationary:

```python
if |velocity| < threshold:
    if dither_flag:
        τ_friction = +gain × |τ_gravity|  
    else:
        τ_friction = -gain × |τ_gravity|
    dither_flag = !dither_flag  # Alternate each cycle
```

This creates a tiny oscillation that prevents the joint from "sticking."

### 5. Joint Limit Barriers

To prevent hitting mechanical limits, a virtual spring-damper pushes the arm away from limits:

```python
if joint_angle > upper_limit:
    τ_barrier = -kp × (angle - limit) - kd × velocity
if joint_angle < lower_limit:
    τ_barrier = -kp × (angle - limit) - kd × velocity
```

### 6. Torque → Current Mapping

Each motor type has a different torque constant:

```python
# XL330-M288-T: Stall torque 0.62Nm at 720mA → ~1158.73 mA/Nm
# XM430-W350-T: Stall torque 4.1Nm at 2300mA → ~561.80 mA/Nm

current_mA = torque_Nm × torque_constant[motor_type]
current_mA = clamp(current_mA, -limit, +limit)  # Safety clamp
```

### 7. The Complete Control Step

```python
def control_step():
    # 1. Read sensors
    positions, velocities = dynamixel.read_positions_and_velocities()
    arm_pos = apply_offsets_and_signs(positions)
    arm_vel = apply_signs(velocities)
    
    # 2. Compute torques
    τ = gravity_torque(arm_pos, arm_vel)    # Main: counteract gravity
    τ += friction_torque(arm_vel)           # Optional: overcome stiction
    τ += joint_limit_torque(arm_pos, arm_vel)  # Safety: avoid limits
    
    # 3. Apply
    τ_with_gripper = [τ[0], τ[1], ..., τ[5], 0.0]  # Gripper gets zero
    τ_signed = τ_with_gripper × joint_signs          # Motor direction
    dynamixel.write_torques(τ_signed)
```

---

## Troubleshooting

### Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| "Port not found" | Wrong port path | Run `ls /dev/serial/by-id/` (Linux) or check Device Manager (Windows) |
| "Permission denied" | User not in dialout group | `sudo usermod -aG dialout $USER`, then re-login |
| High calibration error | Arm not in calibration position | Place arm precisely in home position before starting |
| Arm sags despite compensation | Gain too low or URDF masses wrong | Increase `--gain` or adjust URDF `<mass>` values |
| Arm oscillates | Gain too high | Decrease `--gain` by 0.1 |
| Joint moves wrong direction | Wrong joint sign | Flip the sign for that joint in `joint_signs` |
| "dynamixel_sdk not found" | Missing dependency | `pip install dynamixel-sdk` |
| "pinocchio not found" | Missing dependency | `pip install pin` |
| Control loop overruns | CPU too slow for 500Hz | Lower with `--freq 200` |
| Motor overheats | Excessive current | Lower `--gain` or check URDF masses |

### Emergency Stop

Press `Ctrl+C` at any time. The script will:
1. Immediately zero all motor currents
2. Disable torque on all motors
3. Close the serial port

If the script crashes without cleanup, the motors will hold their last commanded current. **Power cycle the U2D2** to reset.

---

## Credits

- Based on the [GELLO](https://github.com/wuphilipp/gello_software) teleoperation framework by Philipp Wu et al.
- Gravity compensation approach from [FACTR Teleop](https://github.com/RaindragonD/factr_teleop/)
- Uses [Pinocchio](https://github.com/stack-of-tasks/pinocchio) for rigid-body dynamics
- Uses [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) for servo communication

## License

MIT License — see [GELLO Software License](https://github.com/wuphilipp/gello_software/blob/main/LICENSE)
