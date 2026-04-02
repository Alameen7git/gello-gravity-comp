# GELLO UR5 Gravity Compensation

Active gravity compensation for a custom GELLO UR5 leader arm using Dynamixel servos.
Makes the arm feel weightless so you can move it effortlessly during teleoperation.

## Hardware

| Joint | Motor ID | Servo Type | Role |
|-------|----------|------------|------|
| J1 | 1 | XL330-M288-T | Shoulder Pan |
| J2 | 2 | XM430-W350-T | Shoulder Lift |
| J3 | 3 | XM430-W350-T | Elbow |
| J4 | 4 | XL330-M288-T | Wrist 1 |
| J5 | 5 | XL330-M288-T | Wrist 2 |
| J6 | 6 | XL330-M288-T | Wrist 3 |
| J7 | 7 | XL330-M077-T | Gripper |

- **Adapter:** Robotis U2D2
- **Protocol:** Dynamixel Protocol 2.0
- **Baudrate:** 57600
- **OS:** Windows (COM port) or Linux (/dev/ttyUSB0)

## Installation

```bash
pip install -r requirements.txt
```

## Quick Start

**Windows:** Double-click `start.bat`

**Manual:**
```bash
python gello_gui.py
```

## GUI Walkthrough

### Step 1 — Hardware Connection
Select your U2D2 serial port (auto-detected). Default: `COM6` on Windows, `/dev/ttyUSB0` on Linux.

### Step 2 — Configuration
Motor IDs `1, 2, 3, 4, 5, 6, 7` and URDF `gello_ur5.urdf` are pre-filled.

### Step 3 — Calibration
Place the arm in the **UR5 home position** before clicking Initialize:

```
Joint 1 (Shoulder Pan):   0°   — pointing straight forward
Joint 2 (Shoulder Lift): -90°  — upper arm pointing straight UP
Joint 3 (Elbow):         +90°  — forearm pointing straight FORWARD
Joint 4 (Wrist 1):       -90°  — wrist pointing down
Joint 5 (Wrist 2):       -90°
Joint 6 (Wrist 3):         0°
Joint 7 (Gripper):       any
```

### Step 4 — Dashboard
- **Gain slider:** Start at 0.4, increase toward 0.8 until arm feels weightless
- **Dry Run:** Reads sensors without applying torque (safe testing)
- **START COMPENSATION:** Begins the 15 Hz gravity compensation loop

## Key Design Decisions

| Decision | Reason |
|----------|--------|
| `joint_signs = [1, 1, -1, 1, 1, 1]` | J3 elbow motor is physically mounted in reverse |
| Gravity torques negated (`-g * ...`) | Matches Pinocchio RNEA sign convention for UR5 |
| Write → Read → Compute loop order | Prevents TX-after-RX serial port failures on Windows USB |
| GUI reads cached positions only | Avoids concurrent serial access from GUI + control threads |
| GroupSyncWrite for torque enable | Avoids `write1ByteTxRx` RX timeout after sync reads |
| 15 Hz control frequency | Reliable limit for 7 motors at 57600 baud on Windows USB |

## Tuning

If a joint is **pushing** (making gravity worse): flip its sign in `joint_signs`.  
If the arm feels **too stiff**: lower the gain.  
If the arm **sags**: raise the gain.

## CLI Usage

```bash
# Dry run (no torque)
python gravity_compensation.py --dry-run --port COM6

# Custom gain
python gravity_compensation.py --port COM6 --gain 0.5
```

## License
MIT
