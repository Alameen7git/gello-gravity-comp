#!/usr/bin/env python3
"""
Standalone Gravity Compensation for Custom GELLO UR5 Arm
=========================================================

A plug-and-play Python script for active gravity compensation on a
custom-redesigned GELLO UR5 leader arm.

Hardware Configuration:
  - Joint 1:    XL330-M288-T  (original)
  - Joint 2:    XM430-W350-T  (upgraded)
  - Joint 3:    XM430-W350-T  (upgraded)
  - Joint 4-6:  XL330-M288-T  (original)
  - Gripper:    XL330-M077-T  (original)

Dependencies:
  pip install dynamixel-sdk pin numpy

Usage:
  python gravity_compensation.py                     # Run with defaults
  python gravity_compensation.py --dry-run           # Test without torque
  python gravity_compensation.py --gain 0.3          # Custom gravity comp gain
  python gravity_compensation.py --port /dev/ttyUSB0 # Custom serial port

Author: Auto-generated for custom GELLO UR5 build
License: MIT
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Optional, Sequence, Tuple

import numpy as np

# ============================================================================
# CONFIGURATION — Edit these values to match your specific hardware setup
# ============================================================================

DEFAULT_CONFIG = {
    # ---- Serial / Hardware ----
    # "port": "/dev/ttyUSB0",           # Serial port for U2D2 (Linux)
    "port": "COM6",                   # U2D2 on Windows
    "baudrate": 57600,

    # ---- Motor IDs (in order from base to gripper) ----
    "motor_ids": [1, 2, 3, 4, 5, 6, 7],

    # ---- Servo types per joint (must match motor_ids order) ----
    "servo_types": [
        "XL330_M288_T",   # ID 1 - J1 Shoulder Pan
        "XM430_W350_T",   # ID 2 - J2 Shoulder Lift (UPGRADED)
        "XM430_W350_T",   # ID 3 - J3 Elbow (UPGRADED)
        "XL330_M288_T",   # ID 4 - J4 Wrist 1
        "XL330_M288_T",   # ID 5 - J5 Wrist 2
        "XL330_M288_T",   # ID 6 - J6 Wrist 3
        "XL330_M077_T",   # ID 7 - Gripper
    ],

    # ---- Joint sign convention (6 arm joints, gripper excluded) ----
    # From original GELLO repo: J3 elbow motor is physically reversed
    "joint_signs": [1, 1, -1, 1, 1, 1],

    # ---- Number of arm joints (excluding gripper) ----
    "num_arm_joints": 6,

    # ---- URDF path (relative to this script or absolute) ----
    "urdf_path": "gello_ur5.urdf",

    # ---- Control loop frequency (Hz) ----
    # At 57600 baud, 7 motors take ~33ms per read+write cycle → 15Hz is reliable
    "control_frequency_hz": 15,

    # ---- Gravity compensation gain (start low ~0.3, increase to ~0.8) ----
    "gravity_comp_gain": 0.6,

    # ---- Static friction compensation ----
    "friction_comp_gain": 0.0,         # Set >0 to enable (e.g., 0.15)
    "friction_enable_speed": 0.05,     # rad/s threshold

    # ---- Joint limit barrier (repulsive torques near limits) ----
    "joint_limits_max": [ 3.05,  3.05,  3.05,  3.05,  3.05,  3.05],
    "joint_limits_min": [-3.05, -3.05, -3.05, -3.05, -3.05, -3.05],
    "joint_limit_kp": 2.0,
    "joint_limit_kd": 0.1,

    # ---- Calibration position (put arm here before starting) ----
    # UR5 home: J1=0, J2=-90° (up), J3=+90° (forward), J4=-90°, J5=-90°, J6=0°
    "calibration_joint_pos": [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0],

    # ---- Safety ----
    "dry_run": False,                  # True = read only, no torque output
}


# ============================================================================
# MOTOR CONSTANTS
# ============================================================================

# Torque-to-current ratio: mA per Nm
# These are derived from motor datasheets and the GELLO driver
TORQUE_TO_CURRENT = {
    "XL330_M288_T": 1158.73,          # From GELLO: XC330_T288_T mapping
    "XM430_W350_T": 561.80,           # ~1000/1.78 Nm/A at 12V
    "XL330_M077_T": 1158.73,          # Same family as M288
}

# Maximum safe current per motor type (mA)
# Capped below absolute max for safety margin
CURRENT_LIMITS = {
    "XL330_M288_T": 1100,             # Stall ~1193 mA
    "XM430_W350_T": 2000,             # Stall ~2300 mA at 12V
    "XL330_M077_T": 600,              # Lower torque variant
}


# ============================================================================
# DYNAMIXEL SDK CONSTANTS
# ============================================================================

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
CURRENT_CONTROL_MODE = 0
POSITION_CONTROL_MODE = 3


# ============================================================================
# DYNAMIXEL INTERFACE
# ============================================================================

class DynamixelInterface:
    """Lightweight Dynamixel SDK wrapper for multi-motor communication."""

    def __init__(
        self,
        motor_ids: Sequence[int],
        servo_types: Sequence[str],
        port: str,
        baudrate: int = 57600,
    ):
        self.motor_ids = list(motor_ids)
        self.servo_types = list(servo_types)
        self.port = port
        self.baudrate = baudrate
        self.num_motors = len(motor_ids)

        # Build per-motor torque mapping arrays
        self.torque_to_current_map = np.array(
            [TORQUE_TO_CURRENT[s] for s in servo_types], dtype=float
        )
        self.current_limits_arr = np.array(
            [CURRENT_LIMITS[s] for s in servo_types], dtype=float
        )

        self._torque_enabled = False
        self._positions = None          # Latest read (raw int ticks)
        self._velocities = None         # Latest read (raw int units)
        self._lock = Lock()
        self._stop_event = Event()
        self._read_thread: Optional[Thread] = None

        # Will be set in connect()
        self._port_handler = None
        self._packet_handler = None
        self._sync_read = None
        self._sync_write_current = None

    def connect(self) -> None:
        """Open serial port and initialize sync read/write."""
        try:
            from dynamixel_sdk import (
                GroupSyncRead,
                GroupSyncWrite,
                PacketHandler,
                PortHandler,
            )
        except ImportError:
            print("ERROR: dynamixel_sdk not found.")
            print("Install with: pip install dynamixel-sdk")
            sys.exit(1)

        self._port_handler = PortHandler(self.port)
        self._packet_handler = PacketHandler(2.0)  # Protocol 2.0

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {self.port}")
        if not self._port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")

        # Sync read: velocity (4 bytes) + position (4 bytes) = 8 bytes starting at velocity addr
        self._sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION,
        )
        for dxl_id in self.motor_ids:
            if not self._sync_read.addParam(dxl_id):
                raise RuntimeError(f"Failed to add sync read param for motor {dxl_id}")

        # Sync write for current commands
        self._sync_write_current = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

        # Sync write for torque enable (1 byte per motor)
        self._sync_write_torque = GroupSyncWrite(
            self._port_handler,
            self._packet_handler,
            ADDR_TORQUE_ENABLE,
            1,
        )

        print(f"Connected to {self.num_motors} Dynamixel servos on {self.port}")

    def set_operating_mode(self, mode: int) -> None:
        """Set operating mode for all motors. Must be called with torque disabled."""
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        sync_write_mode = None
        with self._lock:
            from dynamixel_sdk import GroupSyncWrite
            self._port_handler.clearPort()
            sync_write_mode = GroupSyncWrite(
                self._port_handler, self._packet_handler, ADDR_OPERATING_MODE, 1
            )
            for dxl_id in self.motor_ids:
                sync_write_mode.addParam(dxl_id, [mode])
            result = sync_write_mode.txPacket()
            sync_write_mode.clearParam()
            if result != COMM_SUCCESS:
                raise RuntimeError(f"Failed to set operating mode (result={result})")
        time.sleep(0.05)
        mode_name = {0: "Current Control", 3: "Position Control"}.get(mode, str(mode))
        print(f"Operating mode set to: {mode_name}")

    def enable_torque(self) -> None:
        """Enable torque on all motors."""
        self._set_torque(True)

    def disable_torque(self) -> None:
        """Disable torque on all motors."""
        self._set_torque(False)

    def _set_torque(self, enable: bool) -> None:
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            self._port_handler.clearPort()
            for dxl_id in self.motor_ids:
                self._sync_write_torque.addParam(dxl_id, [value])
            result = self._sync_write_torque.txPacket()
            self._sync_write_torque.clearParam()
            if result != COMM_SUCCESS:
                print(f"WARNING: Torque {'enable' if enable else 'disable'} sync write failed (result={result})")
        self._torque_enabled = enable
        time.sleep(0.05)  # let motors settle after torque state change

    def read_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """Synchronously read positions (rad) and velocities (rad/s) from motors."""
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        result = self._sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            # Return last known values if available, else zeros
            if self._positions is not None:
                pos_rad = self._positions / 2048.0 * np.pi
                vel_rad_s = self._velocities * 0.229 * 2 * np.pi / 60
                return pos_rad, vel_rad_s
            return np.zeros(self.num_motors), np.zeros(self.num_motors)

        positions = np.zeros(self.num_motors, dtype=np.int32)
        velocities = np.zeros(self.num_motors, dtype=np.int32)

        for i, dxl_id in enumerate(self.motor_ids):
            if self._sync_read.isAvailable(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY):
                vel = self._sync_read.getData(dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
                if vel > 0x7FFFFFFF:
                    vel -= 0x100000000
                velocities[i] = vel
            if self._sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                pos = self._sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                if pos > 0x7FFFFFFF:
                    pos -= 0x100000000
                positions[i] = pos

        self._positions = positions
        self._velocities = velocities
        pos_rad = positions / 2048.0 * np.pi
        vel_rad_s = velocities * 0.229 * 2 * np.pi / 60
        return pos_rad, vel_rad_s

    def write_currents(self, currents_mA: np.ndarray) -> None:
        """Write current commands to all motors via sync write."""
        from dynamixel_sdk.robotis_def import (
            COMM_SUCCESS,
            DXL_HIBYTE,
            DXL_LOBYTE,
        )

        if not self._torque_enabled:
            return

        # Clamp to per-motor limits
        currents_clamped = np.clip(
            currents_mA, -self.current_limits_arr, self.current_limits_arr
        )

        self._port_handler.clearPort()  # flush RX buffer before transmitting
        for dxl_id, current in zip(self.motor_ids, currents_clamped):
            current_int = int(current)
            param = [DXL_LOBYTE(current_int), DXL_HIBYTE(current_int)]
            if not self._sync_write_current.addParam(dxl_id, param):
                print(f"WARNING: Failed to add current param for motor {dxl_id}")
                self._sync_write_current.clearParam()
                return

        result = self._sync_write_current.txPacket()
        if result != COMM_SUCCESS:
            print(f"WARNING: Sync write current failed (result={result})")
        self._sync_write_current.clearParam()

    def write_torques(self, torques_Nm: np.ndarray) -> None:
        """Convert torques (Nm) to currents (mA) and write."""
        currents = self.torque_to_current_map * torques_Nm
        self.write_currents(currents)

    def _start_read_thread(self) -> None:
        self._read_thread = Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def _read_loop(self) -> None:
        """Continuously read positions and velocities in background."""
        from dynamixel_sdk.robotis_def import COMM_SUCCESS

        while not self._stop_event.is_set():
            time.sleep(0.001)
            with self._lock:
                result = self._sync_read.txRxPacket()
                if result != COMM_SUCCESS:
                    continue

                positions = np.zeros(self.num_motors, dtype=np.int32)
                velocities = np.zeros(self.num_motors, dtype=np.int32)

                for i, dxl_id in enumerate(self.motor_ids):
                    # Read velocity
                    if self._sync_read.isAvailable(
                        dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                    ):
                        vel = self._sync_read.getData(
                            dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                        )
                        if vel > 0x7FFFFFFF:
                            vel -= 0x100000000
                        velocities[i] = vel

                    # Read position
                    if self._sync_read.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        pos = self._sync_read.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        if pos > 0x7FFFFFFF:
                            pos -= 0x100000000
                        positions[i] = pos

                self._positions = positions
                self._velocities = velocities

    def close(self) -> None:
        """Stop read thread, disable torque, close port."""
        self._stop_event.set()
        if self._read_thread is not None:
            self._read_thread.join(timeout=2.0)
        if self._port_handler is not None:
            try:
                self.disable_torque()
            except Exception:
                pass
            self._port_handler.closePort()
        print("Dynamixel interface closed.")


# ============================================================================
# GRAVITY COMPENSATOR
# ============================================================================

class GravityCompensator:
    """
    Active gravity compensation for a custom GELLO UR5 arm.

    Uses Pinocchio (pin) for rigid-body inverse dynamics to compute
    the torques needed to counteract gravity, then sends corresponding
    current commands to the Dynamixel servos.
    """

    # Calibration search range: ±20π
    CALIBRATION_RANGE = 20
    CALIBRATION_STEPS = 81

    def __init__(self, config: dict):
        self.config = config
        self.running = False

        # Extract config
        self.num_arm_joints = config["num_arm_joints"]
        self.joint_signs = np.array(config["joint_signs"], dtype=float)
        self.calibration_pos = np.array(config["calibration_joint_pos"], dtype=float)
        self.gravity_comp_gain = config["gravity_comp_gain"]
        self.friction_comp_gain = config["friction_comp_gain"]
        self.friction_enable_speed = config["friction_enable_speed"]
        self.joint_limits_max = np.array(config["joint_limits_max"], dtype=float)
        self.joint_limits_min = np.array(config["joint_limits_min"], dtype=float)
        self.joint_limit_kp = config["joint_limit_kp"]
        self.joint_limit_kd = config["joint_limit_kd"]
        self.dt = 1.0 / config["control_frequency_hz"]
        self.dry_run = config["dry_run"]

        # State
        self.joint_offsets = np.zeros(len(config["motor_ids"]), dtype=float)
        self.tau_g = np.zeros(self.num_arm_joints, dtype=float)
        self.stiction_dither = np.ones(self.num_arm_joints, dtype=bool)

        # Initialize hardware
        self.dxl = DynamixelInterface(
            motor_ids=config["motor_ids"],
            servo_types=config["servo_types"],
            port=config["port"],
            baudrate=config["baudrate"],
        )

        # Initialize Pinocchio model
        self.pin_model = None
        self.pin_data = None

    def initialize(self) -> None:
        """Connect to hardware, load URDF, calibrate."""
        print("=" * 60)
        print("GELLO UR5 Gravity Compensation — Initializing")
        print("=" * 60)

        # Step 1: Connect to Dynamixel servos
        print("\n[1/4] Connecting to Dynamixel servos...")
        self.dxl.connect()

        # Step 2: Load URDF and build Pinocchio model
        print("\n[2/4] Loading URDF for inverse dynamics...")
        self._load_urdf()

        # Step 3: Calibrate joint offsets
        print("\n[3/4] Calibrating joint offsets...")
        self._calibrate()

        # Step 4: Configure servos for current control
        print("\n[4/4] Configuring servos...")
        if not self.dry_run:
            self.dxl.disable_torque()
            self.dxl.set_operating_mode(CURRENT_CONTROL_MODE)
            self.dxl.enable_torque()
            print("Servos set to CURRENT CONTROL mode with torque ENABLED")
        else:
            print("DRY RUN mode — torque will NOT be enabled")

        print("\n" + "=" * 60)
        print("Initialization complete!")
        print(f"  Gravity comp gain: {self.gravity_comp_gain}")
        print(f"  Control frequency: {1/self.dt:.0f} Hz")
        print(f"  Dry run: {self.dry_run}")
        print("=" * 60)

    def _load_urdf(self) -> None:
        """Load URDF and build Pinocchio model - simplified for 2 joints."""
        print("  Simplified 2-joint model (no Pinocchio URDF needed)")
        # For 2 joints, we'll use basic gravity compensation calculations
        # Joint 1: Shoulder pan - no gravity effect (rotation around vertical axis)
        # Joint 2: Shoulder lift - gravity acts on the upper arm
        self.pin_model = None  # Not using Pinocchio
        self.pin_data = None

    def _calibrate(self) -> None:
        """
        Calibrate Dynamixel offsets by matching current raw positions
        to the expected calibration joint positions.

        The arm must be placed in the calibration pose before running.
        """
        # Warm up: take a few readings to stabilize
        for _ in range(10):
            self.dxl.read_positions_and_velocities()

        raw_pos, _ = self.dxl.read_positions_and_velocities()

        # Find offset for each arm joint
        offsets = []
        for i in range(self.num_arm_joints):
            best_offset = 0.0
            best_error = float("inf")

            for offset in np.linspace(
                -self.CALIBRATION_RANGE * np.pi,
                self.CALIBRATION_RANGE * np.pi,
                self.CALIBRATION_STEPS,
            ):
                joint_val = self.joint_signs[i] * (raw_pos[i] - offset)
                error = abs(joint_val - self.calibration_pos[i])
                if error < best_error:
                    best_error = error
                    best_offset = offset
            offsets.append(best_offset)

        self.joint_offsets = np.array(offsets, dtype=float)

        print(f"  Joint offsets: {[f'{x:.3f}' for x in self.joint_offsets]}")

        # Verify calibration
        arm_pos = self._get_arm_positions(raw_pos)
        error = np.abs(arm_pos - self.calibration_pos)
        print(f"  Calibration error: {[f'{x:.4f}' for x in error]}")
        max_error = np.max(error)
        if max_error > 0.3:
            print(f"  WARNING: Max calibration error is {max_error:.3f} rad")
            print("  Make sure the arm is in the calibration position!")

    def _get_arm_positions(self, raw_pos: np.ndarray) -> np.ndarray:
        """Apply offsets and signs to get arm joint positions."""
        n = self.num_arm_joints
        return (raw_pos[:n] - self.joint_offsets[:n]) * self.joint_signs[:n]

    def _get_arm_velocities(self, raw_vel: np.ndarray) -> np.ndarray:
        """Apply signs to get arm joint velocities."""
        n = self.num_arm_joints
        return raw_vel[:n] * self.joint_signs[:n]

    def get_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get calibrated arm joint positions and velocities."""
        raw_pos, raw_vel = self.dxl.read_positions_and_velocities()
        arm_pos = self._get_arm_positions(raw_pos)
        arm_vel = self._get_arm_velocities(raw_vel)
        return arm_pos, arm_vel

    def compute_gravity_torque(
        self, q: np.ndarray, dq: np.ndarray
    ) -> np.ndarray:
        """
        Compute gravity compensation torques for 6-joint GELLO UR5 arm.

        Joint axes (from URDF):
          J1: Z (vertical pan)  → no gravity torque
          J2: Y (shoulder lift) → sagittal plane
          J3: Y (elbow)         → sagittal plane
          J4: Y (wrist 1)       → sagittal plane
          J5: Z (wrist 2)       → orthogonal, small torque
          J6: Y (wrist 3)       → sagittal plane, very small

        Link parameters from gello_ur5.urdf:
          l23=0.1085m, l34=0.0945m, l45=0.0255m, l56=0.0255m
          m2=0.155kg lc2=0.055m, m3=0.145kg lc3=0.05m
          m4=0.048kg lc4=0.015m, m5=0.042kg lc5=0.012m, m6=0.038kg lc6=0.01m
        """
        g = 9.81  # m/s²

        # Link lengths (joint-to-joint, from URDF joint origin z offsets)
        l23 = 0.1085   # J2 → J3
        l34 = 0.0945   # J3 → J4
        l45 = 0.0255   # J4 → J5
        l56 = 0.0255   # J5 → J6

        # Link masses and CoM distances from their proximal joint
        m2, lc2 = 0.155, 0.055
        m3, lc3 = 0.145, 0.050
        m4, lc4 = 0.048, 0.015
        m5, lc5 = 0.042, 0.012
        m6, lc6 = 0.038, 0.010

        # Cumulative angles in the sagittal plane (J1 is pan, doesn't contribute)
        a2  = q[1]              # J2 angle from horizontal
        a23 = q[1] + q[2]      # J2+J3
        a234= q[1] + q[2] + q[3]  # J2+J3+J4

        # Horizontal lever arms for each link's CoM (cos = horizontal projection)
        # J1 (shoulder pan): no gravity effect
        tau_1 = 0.0

        # J2 (shoulder lift): supports everything from J2 outward
        # Negative sign: gravity acts in positive-q direction, so we oppose with negative torque
        tau_2 = -g * (
            m2 * lc2 * np.cos(a2) +
            m3 * (l23 * np.cos(a2) + lc3 * np.cos(a23)) +
            m4 * (l23 * np.cos(a2) + l34 * np.cos(a23) + lc4 * np.cos(a234)) +
            m5 * (l23 * np.cos(a2) + l34 * np.cos(a23) + l45 * np.cos(a234)) +
            m6 * (l23 * np.cos(a2) + l34 * np.cos(a23) + l45 * np.cos(a234))
        )

        # J3 (elbow): supports everything from J3 outward
        tau_3 = -g * (
            m3 * lc3 * np.cos(a23) +
            m4 * (l34 * np.cos(a23) + lc4 * np.cos(a234)) +
            m5 * (l34 * np.cos(a23) + l45 * np.cos(a234)) +
            m6 * (l34 * np.cos(a23) + l45 * np.cos(a234))
        )

        # J4 (wrist 1): supports wrist links
        tau_4 = -g * (
            m4 * lc4 * np.cos(a234) +
            m5 * l45 * np.cos(a234) +
            m6 * l45 * np.cos(a234)
        )

        # J5 (wrist 2, Z-axis): gravity torque is 0 when wrist is symmetric;
        # small cross-coupling term — approximate as zero
        tau_5 = 0.0

        # J6 (wrist 3): very small, links are tiny
        tau_6 = 0.0

        self.tau_g = np.array([tau_1, tau_2, tau_3, tau_4, tau_5, tau_6]) * self.gravity_comp_gain
        return self.tau_g

    def compute_friction_torque(self, dq: np.ndarray) -> np.ndarray:
        """
        Static friction compensation using dithering.

        When joint velocity is near zero, apply small alternating torques
        to overcome stiction (static friction) in the servos/gears.
        """
        if self.friction_comp_gain <= 0:
            return np.zeros(self.num_arm_joints)

        tau_f = np.zeros(self.num_arm_joints)
        for i in range(self.num_arm_joints):
            if abs(dq[i]) < self.friction_enable_speed:
                if self.stiction_dither[i]:
                    tau_f[i] = self.friction_comp_gain * abs(self.tau_g[i])
                else:
                    tau_f[i] = -self.friction_comp_gain * abs(self.tau_g[i])
                self.stiction_dither[i] = not self.stiction_dither[i]
        return tau_f

    def compute_joint_limit_torque(
        self, q: np.ndarray, dq: np.ndarray
    ) -> np.ndarray:
        """
        Repulsive barrier torques near joint limits.

        Pushes the arm away from limits with a PD-like spring-damper.
        """
        tau_l = np.zeros(self.num_arm_joints)

        # Exceeded upper limit
        exceed_max = q > self.joint_limits_max
        if np.any(exceed_max):
            tau_l += (
                -self.joint_limit_kp * (q - self.joint_limits_max)
                - self.joint_limit_kd * dq
            ) * exceed_max

        # Exceeded lower limit
        exceed_min = q < self.joint_limits_min
        if np.any(exceed_min):
            tau_l += (
                -self.joint_limit_kp * (q - self.joint_limits_min)
                - self.joint_limit_kd * dq
            ) * exceed_min

        return tau_l

    def control_step(self) -> None:
        """Execute one step of the gravity compensation control loop.

        Order: WRITE (port clean) → READ (leaves port dirty) → COMPUTE
        The next iteration's write happens before any read, avoiding TX-after-RX failures.
        """
        # 1. Send torques computed in the previous step (port is clean here)
        if not self.dry_run and hasattr(self, '_pending_torque'):
            self.dxl.write_torques(self._pending_torque)

        # 2. Read current joint state (may leave RX buffer with residual bytes)
        arm_pos, arm_vel = self.get_joint_states()

        # 3. Compute torques for next iteration
        torque_arm = np.zeros(self.num_arm_joints)
        torque_arm += self.compute_gravity_torque(arm_pos, arm_vel)
        torque_arm += self.compute_friction_torque(arm_vel)
        torque_arm += self.compute_joint_limit_torque(arm_pos, arm_vel)

        arm_signed = torque_arm * self.joint_signs
        self._pending_torque = np.append(arm_signed, 0.0)  # gripper = 0

        if self.dry_run:
            if not hasattr(self, '_log_counter'):
                self._log_counter = 0
            self._log_counter += 1
            if self._log_counter % (20 * 5) == 0:  # every 5 seconds at 20Hz
                print(f"\n[DRY RUN] Positions (deg): "
                      f"{[f'{np.degrees(x):7.1f}' for x in arm_pos]}")
                print(f"[DRY RUN] Gravity τ (Nm):  "
                      f"{[f'{x:7.4f}' for x in self.tau_g]}")

    def run(self) -> None:
        """Main control loop."""
        print(f"\nStarting gravity compensation at {1/self.dt:.0f} Hz")
        print("Press Ctrl+C to stop\n")

        self.running = True
        loop_count = 0
        overrun_count = 0

        try:
            while self.running:
                t0 = time.time()

                self.control_step()

                elapsed = time.time() - t0
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    overrun_count += 1
                    if overrun_count % 100 == 1:
                        print(f"WARNING: Loop overrun by {-sleep_time*1000:.1f}ms "
                              f"(total overruns: {overrun_count})")

                loop_count += 1

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            self.shutdown()

        print(f"Ran {loop_count} control loops, {overrun_count} overruns")

    def shutdown(self) -> None:
        """Safely shutdown: zero torques, disable motors, close connection."""
        self.running = False
        print("\nShutting down...")

        if not self.dry_run:
            try:
                # Send zero torques first
                zero_torque = np.zeros(len(self.config["motor_ids"]))
                self.dxl.write_currents(zero_torque)
                time.sleep(0.05)
            except Exception as e:
                print(f"Warning during zero torque: {e}")

        self.dxl.close()
        print("Shutdown complete.")


# ============================================================================
# MAIN
# ============================================================================

def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Gravity Compensation for Custom GELLO UR5 Arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python gravity_compensation.py                          # Run with defaults
  python gravity_compensation.py --dry-run                # Test without torque
  python gravity_compensation.py --gain 0.3               # Low gain (safe start)
  python gravity_compensation.py --port /dev/ttyUSB0      # Specify port
  python gravity_compensation.py --port COM3              # Windows port
  python gravity_compensation.py --freq 200               # Lower control rate

Calibration:
  Before starting, place the arm in the calibration position:
  [0, -π/2, π/2, -π/2, -π/2, 0] (standard GELLO UR5 home)

Safety:
  Always start with --dry-run first to verify sensor readings.
  Then start with --gain 0.2 and gradually increase.
        """
    )
    parser.add_argument(
        "--port", "-p", type=str, default=None,
        help="Serial port (e.g., /dev/ttyUSB0 or COM3)"
    )
    parser.add_argument(
        "--gain", "-g", type=float, default=None,
        help="Gravity compensation gain (0.0 to 1.0, default: 0.6)"
    )
    parser.add_argument(
        "--freq", "-f", type=int, default=None,
        help="Control loop frequency in Hz (default: 500)"
    )
    parser.add_argument(
        "--dry-run", "-d", action="store_true",
        help="Dry run mode: read sensors only, no torque output"
    )
    parser.add_argument(
        "--urdf", "-u", type=str, default=None,
        help="Path to URDF file (default: gello_ur5.urdf)"
    )
    parser.add_argument(
        "--friction", type=float, default=None,
        help="Friction compensation gain (default: 0.0)"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Build config from defaults + CLI overrides
    config = DEFAULT_CONFIG.copy()
    if args.port is not None:
        config["port"] = args.port
    if args.gain is not None:
        config["gravity_comp_gain"] = args.gain
    if args.freq is not None:
        config["control_frequency_hz"] = args.freq
    if args.dry_run:
        config["dry_run"] = True
    if args.urdf is not None:
        config["urdf_path"] = args.urdf
    if args.friction is not None:
        config["friction_comp_gain"] = args.friction

    try:
        comp = GravityCompensator(config)

        # Setup signal handlers
        def signal_handler(signum, frame):
            print("\nReceived shutdown signal")
            comp.running = False

        signal.signal(signal.SIGINT, signal_handler)
        if hasattr(signal, 'SIGTERM'):
            signal.signal(signal.SIGTERM, signal_handler)

        comp.initialize()
        comp.run()

    except FileNotFoundError as e:
        print(f"\nERROR: {e}")
        return 1
    except RuntimeError as e:
        print(f"\nERROR: {e}")
        return 1
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
