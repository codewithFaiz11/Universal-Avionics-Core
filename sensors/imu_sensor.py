"""
6-DOF IMU Sensor Driver (Generic)
Generalised for Universal Avionics Core

Calculates integrated XYZ position from raw acceleration data.
Designed as a template for I2C/Serial IMUs (e.g., MPU6050, BNO055) 
or Network-based testing.
"""
import time
import math
import numpy as np
import urllib.request
import json

class SensorDriver:
    def __init__(self):
        print("[HARDWARE] Initializing 6-DOF IMU Sensor...")
        
        # --- HARDWARE CONFIGURATION ---
        # TODO: Replace with real hardware initialization (e.g., SMBus for I2C, Serial for USB)
        
        # For development/testing, keeping a network interface fallback:
        self.use_network_imu = True
        self.network_url = "http://<YOUR_DEVICE_IP>:8080/get?accX&accY&accZ" 
        
        # Physics State Variables
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])
        self.last_time = time.time()
        
        self.base_gravity = np.array([0.0, 0.0, 0.0])
        self.calibrate_hardware()

    def calibrate_hardware(self):
        """Captures the resting gravity vector for zero-bias."""
        print("[SYSTEM] Calibrating IMU... Keep sensor completely FLAT and STILL!")
        samples = []
        for _ in range(20):
            raw = self._read_raw_accel()
            if raw is not None:
                samples.append(raw)
            time.sleep(0.1)
        
        if samples:
            self.base_gravity = np.mean(samples, axis=0)
            print(f"[SUCCESS] Gravity bias locked at: X:{self.base_gravity[0]:.2f} Y:{self.base_gravity[1]:.2f} Z:{self.base_gravity[2]:.2f}")
        else:
            print("[ERROR] Failed to read IMU. Check I2C/Serial/Network connections.")

    def _read_raw_accel(self):
        """Fetches raw acceleration. Replace this block with actual I2C/Serial reads."""
        if self.use_network_imu:
            try:
                # Fallback testing logic
                req = urllib.request.urlopen(self.network_url.replace("<YOUR_DEVICE_IP>", "172.26.184.232"), timeout=1)
                data = json.loads(req.read().decode())
                if 'accX' in data['buffer']:
                    ax = data['buffer']['accX']['buffer'][0]
                    ay = data['buffer']['accY']['buffer'][0]
                    az = data['buffer']['accZ']['buffer'][0]
                    return np.array([ax, ay, az])
            except Exception:
                return None
        else:
            # ---> REAL HARDWARE LOGIC GOES HERE <---
            # Example: 
            # accel_data = i2c_bus.read_i2c_block_data(DEVICE_ADDRESS, ACCEL_XOUT_H, 6)
            # return np.array([x, y, z])
            pass
        return None

    def get_vector(self):
        """Returns the canonical [X, Y, Z] spatial array required by the Avionics Core."""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        raw_acc = self._read_raw_accel()
        if raw_acc is None:
            return None

        # 1. Strip gravity bias and apply deadband filter
        net_acc = raw_acc - self.base_gravity
        net_acc[np.abs(net_acc) < 0.2] = 0.0  # Noise floor threshold
        
        # 2. Double Integration: Acceleration -> Velocity -> Position
        self.velocity += net_acc * dt
        self.velocity *= 0.92  # Velocity dampener to prevent dead-reckoning runaway
        self.position += self.velocity * dt

        # 3. Calculate Live Dynamic Mount Angles (Roll, Pitch, Yaw)
        raw_ax, raw_ay, raw_az = raw_acc
        roll = math.degrees(math.atan2(raw_ay, raw_az))
        pitch = math.degrees(math.atan2(-raw_ax, math.sqrt(raw_ay**2 + raw_az**2)))
        yaw = 0.0 

        # Return standard [X, Y, Z] position output AND Attitude
        return self.position, np.array([roll, pitch, yaw])

    def close(self):
        print("[HARDWARE] IMU safely disconnected.")