"""
Universal Avionics Core Middleware
Translates spatial sensor data into a canonical axis frame for MAVLink injection.
Supports both Static (Camera/JSON) and Dynamic (IMU) Mounting Angles.
"""
import numpy as np
import math
import time
import threading
import json
import os
import sys
from pymavlink import mavutil

# --- HARDWARE ABSTRACTION LAYER (HAL) ---
# CHANGE THE IMPORT BELOW TO SWITCH SENSORS:
# from sensors.sensor_camera import SensorDriver 
from sensors.imu_sensor import SensorDriver 
from calibration.ultimate_calibration import run_calibration

from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.console import Console
from rich import box

# System State Variables
latest_v_raw = [0.0, 0.0, 0.0]
latest_v_canonical = [0.0, 0.0, 0.0]
last_frame_time = 0.0
system_running = True
watchdog_tripped = False
lag_time = 0.0

# Mount State Variables
R_MOUNT_MATRIX = None
SENSOR_MODE = "WAITING..."

# --- NON-VOLATILE MEMORY SYSTEM ---
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "mount_config.json")

def load_config():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, 'r') as f:
            return json.load(f)
    return {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

def save_config(roll, pitch, yaw):
    os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
    with open(CONFIG_FILE, 'w') as f:
        json.dump({"roll": roll, "pitch": pitch, "yaw": yaw}, f)

config = load_config()
MOUNT_ROLL_DEG, MOUNT_PITCH_DEG, MOUNT_YAW_DEG = config['roll'], config['pitch'], config['yaw']

def generate_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return np.dot(Rz, np.dot(Ry, Rx)).T

def sensor_loop():
    global latest_v_raw, latest_v_canonical, last_frame_time, system_running
    global MOUNT_ROLL_DEG, MOUNT_PITCH_DEG, MOUNT_YAW_DEG, R_MOUNT_MATRIX, SENSOR_MODE
    
    sensor = SensorDriver()

    while system_running:
        data = sensor.get_vector()
        if data is None: continue
        
        # Unpack the Dual Payload (Position, Attitude)
        V_raw, live_attitude = data
        latest_v_raw = V_raw
        
        # DYNAMIC MOUNT LOGIC
        if live_attitude is not None:
            MOUNT_ROLL_DEG, MOUNT_PITCH_DEG, MOUNT_YAW_DEG = live_attitude
            R_MOUNT_MATRIX = generate_rotation_matrix(MOUNT_ROLL_DEG, MOUNT_PITCH_DEG, MOUNT_YAW_DEG)
            SENSOR_MODE = "[bold yellow]DYNAMIC (IMU)[/bold yellow]"
        else:
            SENSOR_MODE = "[bold blue]STATIC (JSON)[/bold blue]"
            # R_MOUNT_MATRIX is pre-calculated in __main__ for static sensors
        
        # Sensor Frame to Aero Frame Mapping (Assumes standard Depth/Right/Down input)
        raw_x, raw_y, raw_z = V_raw[0], V_raw[1], V_raw[2]
        V_aligned_to_drone = np.array([raw_z, raw_x, raw_y])
        
        # Apply physical mount rotation
        latest_v_canonical = np.dot(R_MOUNT_MATRIX, V_aligned_to_drone)
        last_frame_time = time.time()
        
    sensor.close()

def mavlink_loop():
    global latest_v_canonical, last_frame_time, system_running, watchdog_tripped, lag_time
    try:
        # Jetson MAVLink connection
        master = mavutil.mavlink_connection('udpin:127.0.0.1:14551', source_system=254)
    except Exception as e:
        print(f"[ERROR] MAVLink Connection Failed: {e}")
        system_running = False
        return

    last_heartbeat_time = 0.0
    while system_running:
        current_time = time.time()
        if current_time - last_heartbeat_time >= 1.0:
            master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_heartbeat_time = current_time

        lag_time = current_time - last_frame_time
        if lag_time > 0.4:
            watchdog_tripped = True
        else:
            watchdog_tripped = False
            master.mav.vision_position_estimate_send(
                int(current_time * 1e6),
                latest_v_canonical[0], latest_v_canonical[1], latest_v_canonical[2],
                0, 0, 0
            )
        time.sleep(0.05) 

if __name__ == '__main__':
    console = Console()
    console.print(Panel("[bold cyan]UNIVERSAL AVIONICS CORE INITIALIZATION[/bold cyan]", expand=False))
    print(f"Stored JSON Angles -> Pitch: {MOUNT_PITCH_DEG:.2f}°, Roll: {MOUNT_ROLL_DEG:.2f}°")
    
    choice = input("Press [ENTER] to fly, or type 'c' to recalibrate (ArUco Camera Only): ").strip().lower()
    
    if choice == 'c':
        new_roll, new_pitch, new_yaw = run_calibration()
        if new_roll is not None:
            MOUNT_ROLL_DEG, MOUNT_PITCH_DEG, MOUNT_YAW_DEG = new_roll, new_pitch, new_yaw
            save_config(new_roll, new_pitch, new_yaw)
            print("[SUCCESS] New calibration locked to Jetson memory.\n")
        else:
            print("[WARNING] Calibration aborted. Using old values.\n")
    
    # Pre-calculate matrix for Static sensors
    R_MOUNT_MATRIX = generate_rotation_matrix(MOUNT_ROLL_DEG, MOUNT_PITCH_DEG, MOUNT_YAW_DEG)

    print("[SYSTEM] Starting Main Flight Threads...")
    t_sensor = threading.Thread(target=sensor_loop)
    t_mavlink = threading.Thread(target=mavlink_loop)
    t_sensor.start()
    t_mavlink.start()
    time.sleep(1.5) 
    start_time = time.time()

    def build_dashboard():
        uptime = int(time.time() - start_time)
        lag = lag_time
        status_text = "[bold green]HEALTHY – INJECTING[/bold green]" if not watchdog_tripped else "[bold red]FAILSAFE TRIGGERED[/bold red]"
        bar_len = 20
        filled = int(min(lag / 0.5, 1.0) * bar_len) 
        bar_colour = "green" if lag <= 0.4 else "red"
        bar = f"[{bar_colour}]{'█' * filled + '░' * (bar_len - filled)}[/{bar_colour}]"

        table = Table(show_header=False, box=box.ROUNDED, padding=(0, 1))
        table.add_column(style="bold cyan", width=14)
        table.add_column(style="white")
        table.add_row("System Status", status_text)
        table.add_row("Uptime", f"{uptime // 60:02d}:{uptime % 60:02d}")
        table.add_row("Sensor Lag", f"{lag:.3f}s  {bar}")
        table.add_row("")
        
        # Dynamic Mount Display Line
        table.add_row("MOUNT CONFIG", f"P: {MOUNT_PITCH_DEG:>6.2f}° | R: {MOUNT_ROLL_DEG:>6.2f}°  {SENSOR_MODE}")
        
        table.add_row("RAW SENSOR", f"RawX: {latest_v_raw[0]:>6.2f}  RawY: {latest_v_raw[1]:>6.2f}  RawZ: {latest_v_raw[2]:>6.2f}")
        table.add_row("", "[dim]↕ ALIGNMENT & SPATIAL ROTATION[/dim]")
        table.add_row("DRONE EKF", f"Fwd(X): {latest_v_canonical[0]:>6.2f}  Right(Y): {latest_v_canonical[1]:>6.2f}  Down(Z): {latest_v_canonical[2]:>6.2f}")

        panel = Panel(table, title="[bold magenta]SURGE 2026 • UNIVERSAL AVIONICS CORE[/bold magenta]", border_style="bright_blue", padding=(1, 2))
        return panel

    try:
        with Live(build_dashboard(), refresh_per_second=10, screen=True) as live:
            while system_running:
                live.update(build_dashboard())
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        system_running = False
        t_sensor.join()
        t_mavlink.join()
        console.print("\n[bold green]System safely shut down.[/bold green]")