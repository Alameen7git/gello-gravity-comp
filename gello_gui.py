#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import sys
import os
from pathlib import Path
import numpy as np

# Import the logic from our main script
try:
    from gravity_compensation import GravityCompensator, DEFAULT_CONFIG
except ImportError:
    messagebox.showerror("Error", "Could not find gravity_compensation.py in the current directory.")
    sys.exit(1)

class GelloGui:
    def __init__(self, root):
        self.root = root
        self.root.title("GELLO UR5 Gravity Compensation Setup")
        self.root.geometry("700x600")
        self.root.configure(bg="#1e1e1e")
        
        self.style = ttk.Style()
        self.style.theme_use("clam")
        
        # Configure dark theme colors
        self.style.configure("TFrame", background="#1e1e1e")
        self.style.configure("TLabel", background="#1e1e1e", foreground="#ffffff", font=("Segoe UI", 10))
        self.style.configure("Header.TLabel", font=("Segoe UI", 16, "bold"), foreground="#00a8e8")
        self.style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=10)
        self.style.configure("Accent.TButton", foreground="#ffffff", background="#007acc")
        
        self.comp = None
        self.is_running = False
        self.current_step = 0
        
        # Setup modern frames for step-by-step
        self.container = ttk.Frame(self.root)
        self.container.pack(fill="both", expand=True, padx=40, pady=40)
        
        self.steps = [
            self.setup_step_1, # Welcome & Port
            self.setup_step_2, # Motor IDs & URDF
            self.setup_step_3, # Calibration Position
            self.setup_step_4, # Control Dashboard
        ]
        
        self.show_step(0)

    def clear_container(self):
        for widget in self.container.winfo_children():
            widget.destroy()

    def show_step(self, index):
        self.current_step = index
        self.clear_container()
        self.steps[index]()

    def setup_step_1(self):
        """Step 1: Serial Port Selection"""
        ttk.Label(self.container, text="STEP 1: Hardware Connection", style="Header.TLabel").pack(pady=(0, 20))
        ttk.Label(self.container, text="Select the serial port for your U2D2 adapter.").pack(pady=5)
        
        port_frame = ttk.Frame(self.container)
        port_frame.pack(pady=10)
        
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not ports:
            ports = ["No ports found"]
            
        self.port_var = tk.StringVar(value=DEFAULT_CONFIG["port"] if DEFAULT_CONFIG["port"] in ports else ports[0])
        port_menu = ttk.Combobox(port_frame, textvariable=self.port_var, values=ports, width=30)
        port_menu.pack(side="left", padx=5)
        
        refresh_btn = ttk.Button(port_frame, text="Refresh", command=lambda: self.show_step(0))
        refresh_btn.pack(side="left")
        
        ttk.Label(self.container, text="Baudrate (default 57600):").pack(pady=(15, 5))
        self.baud_var = tk.StringVar(value=str(DEFAULT_CONFIG["baudrate"]))
        ttk.Entry(self.container, textvariable=self.baud_var, width=15).pack()
        
        btn_frame = ttk.Frame(self.container)
        btn_frame.pack(side="bottom", fill="x", pady=20)
        ttk.Button(btn_frame, text="Next →", style="Accent.TButton", command=lambda: self.show_step(1)).pack(side="right")

    def setup_step_2(self):
        """Step 2: Motor IDs & URDF Check"""
        ttk.Label(self.container, text="STEP 2: Configuration", style="Header.TLabel").pack(pady=(0, 20))
        
        ttk.Label(self.container, text="Verify Motor IDs (usually 1-7 from base to gripper):").pack(pady=5)
        self.ids_var = tk.StringVar(value=", ".join(map(str, DEFAULT_CONFIG["motor_ids"])))
        ttk.Entry(self.container, textvariable=self.ids_var, width=40).pack(pady=5)
        
        ttk.Label(self.container, text="URDF File (must be in root folder):").pack(pady=(15, 5))
        self.urdf_var = tk.StringVar(value=DEFAULT_CONFIG["urdf_path"])
        ttk.Entry(self.container, textvariable=self.urdf_var, width=40).pack(pady=5)
        
        # Check if URDF exists (resolve relative to script dir)
        script_dir = Path(__file__).parent
        urdf_path = Path(self.urdf_var.get())
        if not urdf_path.is_absolute():
            urdf_path = script_dir / urdf_path
        if urdf_path.exists():
            ttk.Label(self.container, text="✅ URDF Found", foreground="#4caf50").pack()
        else:
            ttk.Label(self.container, text=f"❌ URDF Not Found: {urdf_path}", foreground="#f44336").pack()
            
        btn_frame = ttk.Frame(self.container)
        btn_frame.pack(side="bottom", fill="x", pady=20)
        ttk.Button(btn_frame, text="← Back", command=lambda: self.show_step(0)).pack(side="left")
        ttk.Button(btn_frame, text="Next →", style="Accent.TButton", command=lambda: self.show_step(2)).pack(side="right")

    def setup_step_3(self):
        """Step 3: Calibration Instructions"""
        ttk.Label(self.container, text="STEP 3: Calibration", style="Header.TLabel").pack(pady=(0, 20))

        instr = (
            "Place your GELLO arm in the UR5 HOME position:\n\n"
            "• Joint 1 (ID 1 - Shoulder Pan):  0°   (pointing straight FORWARD)\n"
            "• Joint 2 (ID 2 - Shoulder Lift): -90° (upper arm pointing straight UP)\n"
            "• Joint 3 (ID 3 - Elbow):         +90° (forearm pointing FORWARD)\n"
            "• Joint 4 (ID 4 - Wrist 1):       -90° (wrist pointing DOWN)\n"
            "• Joint 5 (ID 5 - Wrist 2):       -90°\n"
            "• Joint 6 (ID 6 - Wrist 3):         0°\n"
            "• Joint 7 (ID 7 - Gripper):   any pos (not used for calibration)\n\n"
            "Hold the arm steady in this pose, then click INITIALIZE ARM."
        )
        ttk.Label(self.container, text=instr, justify="left").pack(pady=10)
        
        btn_frame = ttk.Frame(self.container)
        btn_frame.pack(side="bottom", fill="x", pady=20)
        ttk.Button(btn_frame, text="← Back", command=lambda: self.show_step(1)).pack(side="left")
        
        self.init_btn = ttk.Button(btn_frame, text="INITIALIZE ARM ⭐", style="Accent.TButton", command=self.initialize_arm)
        self.init_btn.pack(side="right")

    def initialize_arm(self):
        # Update config from GUI variables
        config = DEFAULT_CONFIG.copy()
        config["port"] = self.port_var.get()
        config["baudrate"] = int(self.baud_var.get())
        config["motor_ids"] = [int(x.strip()) for x in self.ids_var.get().split(",")]
        # Resolve URDF path relative to script dir so it works from any CWD
        urdf = Path(self.urdf_var.get())
        if not urdf.is_absolute():
            urdf = Path(__file__).parent / urdf
        config["urdf_path"] = str(urdf)
        
        self.init_btn.configure(state="disabled", text="Initializing...")
        
        def do_init():
            try:
                self.comp = GravityCompensator(config)
                self.comp.initialize()
                self.root.after(0, lambda: self.show_step(3))
            except Exception as e:
                self.root.after(0, lambda: messagebox.showerror("Init Failed", str(e)))
                self.root.after(0, lambda: self.init_btn.configure(state="normal", text="INITIALIZE ARM ⭐"))

        threading.Thread(target=do_init, daemon=True).start()

    def setup_step_4(self):
        """Step 4: Control Dashboard"""
        ttk.Label(self.container, text="GRAVITY COMPENSATION DASHBOARD", style="Header.TLabel").pack(pady=(0, 20))
        
        # Gain Control
        gain_frame = ttk.Frame(self.container)
        gain_frame.pack(fill="x", pady=5)
        ttk.Label(gain_frame, text="Gravity Compensation Gain:").pack(side="left")
        self.gain_val_label = ttk.Label(gain_frame, text=f"{self.comp.gravity_comp_gain:.2f}")
        self.gain_val_label.pack(side="right")
        
        self.gain_slider = ttk.Scale(self.container, from_=0, to=1.0, value=self.comp.gravity_comp_gain, command=self.update_gain)
        self.gain_slider.pack(fill="x", pady=(0, 20))
        
        # Dry Run Toggle
        self.dry_run_var = tk.BooleanVar(value=self.comp.dry_run)
        dry_run_chk = ttk.Checkbutton(self.container, text="Dry Run (No physical motor torque)", variable=self.dry_run_var, command=self.update_dry_run)
        dry_run_chk.pack(anchor="w", pady=5)
        
        # Live Status
        status_frame = tk.Frame(self.container, bg="#2d2d2d", padx=10, pady=10)
        status_frame.pack(fill="both", expand=True, pady=10)
        
        self.status_text = tk.Text(status_frame, bg="#2d2d2d", fg="#00ff00", font=("Consolas", 10), height=8, borderwidth=0)
        self.status_text.pack(fill="both", expand=True)
        self.status_text.insert("1.0", "System Ready.\nPress START to begin compensation loop.")
        
        # Control Buttons
        btn_frame = ttk.Frame(self.container)
        btn_frame.pack(side="bottom", fill="x", pady=20)
        
        self.start_btn = ttk.Button(btn_frame, text="START COMPENSATION", style="Accent.TButton", command=self.toggle_run)
        self.start_btn.pack(side="right", padx=5)
        
        ttk.Button(btn_frame, text="Shutdown & Exit", command=self.on_closing).pack(side="left")
        
        # Start status update loop
        self.update_status()

    def update_gain(self, val):
        val = float(val)
        self.comp.gravity_comp_gain = val
        self.gain_val_label.configure(text=f"{val:.2f}")

    def update_dry_run(self):
        self.comp.dry_run = self.dry_run_var.get()
        if self.is_running:
            # If running, we might need to toggle physical torque
            if self.comp.dry_run:
                self.comp.dxl.disable_torque()
            else:
                self.comp.dxl.enable_torque()

    def toggle_run(self):
        if not self.is_running:
            self.is_running = True
            self.start_btn.configure(text="■ STOP")
            threading.Thread(target=self.comp_loop, daemon=True).start()
        else:
            self.is_running = False
            self.start_btn.configure(text="START COMPENSATION")
            self.comp.running = False

    def comp_loop(self):
        try:
            self.comp.run()
        except Exception as e:
            self.is_running = False
            self.root.after(0, lambda: messagebox.showerror("Crash", str(e)))
            self.root.after(0, lambda: self.start_btn.configure(text="START COMPENSATION"))

    def update_status(self):
        if self.comp and hasattr(self.comp, 'tau_g'):
            try:
                # Read from cached positions — never touch the serial port from the GUI thread
                dxl = self.comp.dxl
                n = self.comp.num_arm_joints
                if dxl._positions is None:
                    return
                raw_rad = dxl._positions[:n] / 2048.0 * np.pi
                arm_pos = (raw_rad - self.comp.joint_offsets[:n]) * self.comp.joint_signs[:n]
                status = "Robot Live Status:\n"
                status += "-" * 40 + "\n"
                status += f"{'Joint':<10} | {'Pos (Deg)':<12} | {'Torque (Nm)':<12}\n"
                for i in range(self.comp.num_arm_joints):
                    status += f"Joint {i+1:<4} | {np.degrees(arm_pos[i]):>10.2f} | {self.comp.tau_g[i]:>11.4f}\n"
                
                self.status_text.delete("1.0", tk.END)
                self.status_text.insert("1.0", status)
            except:
                pass
                
        if self.current_step == 3:
            self.root.after(100, self.update_status)

    def on_closing(self):
        if self.comp:
            self.comp.shutdown()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    # Centering the window
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width/2) - (700/2)
    y = (screen_height/2) - (600/2)
    root.geometry(f'700x600+{int(x)}+{int(y)}')
    
    gui = GelloGui(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()

