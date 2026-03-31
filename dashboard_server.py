#!/usr/bin/env python3
import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template, send_from_directory
from flask_socketio import SocketIO, emit
import threading
import time
import os
import sys
import numpy as np

# Import our robot logic
try:
    from gravity_compensation import GravityCompensator, DEFAULT_CONFIG
except ImportError:
    print("Error: Could not find gravity_compensation.py.")
    sys.exit(1)

app = Flask(__name__, static_folder='static')
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# Shared robot instance
comp = None
running_thread = None

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/<path:path>')
def send_static(path):
    return send_from_directory('static', path)

@socketio.on('connect')
def handle_connect():
    print("Dashboard connected")
    emit('notification', {'message': 'Dashboard linked to Robot Server'})

@socketio.on('connect_hardware')
def handle_connect_hw():
    global comp
    try:
        if comp is None:
            config = DEFAULT_CONFIG.copy()
            # In a real environment, we'd take this from the UI
            comp = GravityCompensator(config)
            comp.initialize()
            emit('notification', {'message': 'Hardware Connected & Ready'})
        else:
            emit('notification', {'message': 'Hardware already active'})
    except Exception as e:
        emit('notification', {'message': f"Error: {str(e)}", 'type': 'error'})

@socketio.on('calibrate')
def handle_calibrate():
    if comp:
        try:
            comp._calibrate()
            emit('notification', {'message': 'Joints Synchronized'})
        except Exception as e:
            emit('notification', {'message': f"Sync Error: {str(e)}"})

@socketio.on('update_gain')
def handle_gain(data):
    if comp:
        comp.gravity_comp_gain = float(data['value'])

@socketio.on('toggle_run')
def handle_toggle():
    global comp, running_thread
    if comp:
        if not comp.running:
            comp.running = True
            running_thread = threading.Thread(target=comp.run, daemon=True)
            running_thread.start()
            print("Gravity Compensation Loop Started")
        else:
            comp.running = False
            print("Gravity Compensation Loop Stopped")

def telemetry_broadcast():
    """High-speed telemetry pusher (60Hz)"""
    while True:
        if comp and comp.dxl and hasattr(comp, 'tau_g'):
            try:
                # Read current states
                q, dq = comp.get_joint_states()
                
                # Push degrees to dashboard
                socketio.emit('telemetry', {
                    'q': np.degrees(q).tolist(),
                    'tau': comp.tau_g.tolist()
                })
            except Exception:
                pass
        socketio.sleep(0.016) # 60Hz update rate

# Start telemetry loop in background
socketio.start_background_task(telemetry_broadcast)

if __name__ == '__main__':
    print("=" * 60)
    print("GELLO UR5 NEXT-GEN DASHBOARD")
    print("=" * 60)
    print("Access the Graphic Interface at: http://127.0.0.1:5000")
    print("=" * 60)
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
