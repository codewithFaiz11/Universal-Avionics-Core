"""
SURGE 2026: Universal Calibration Engine (ArUco Method)
Calculates physical mounting offsets between the Flight Controller and the Vision Sensor.
Uses RQDecomp3x3 to perfectly match live flight core matrices.
"""
import cv2
import numpy as np
import cv2.aruco as aruco
from pymavlink import mavutil
import math
import time

# --- HARDWARE & NETWORK CONFIGURATION ---
PORT = 'udpin:127.0.0.1:14552'  # MAVProxy split port
BAUD_RATE = 115200

# --- VISION SYSTEM CONFIGURATION ---
MARKER_SIZE = 0.091      # Physical size of the ArUco marker in meters
FOCAL_LENGTH = 600.0     # Camera intrinsic focal length (tune for your lens)
CENTER_X = 320.0         # Frame width / 2
CENTER_Y = 240.0         # Frame height / 2

# --- CALIBRATION PARAMETERS ---
REQUIRED_SAMPLES = 30    # Number of valid frames required to lock calibration
NOISE_THRESHOLD = 1.5    # Degrees of deadband to remove micro-vibrations

def snap_to_zero(angle_deg, threshold=NOISE_THRESHOLD):
    """Deadband filter to snap tiny noise values exactly to zero."""
    if abs(angle_deg) <= threshold:
        return 0.0
    return round(angle_deg, 2)

def run_calibration():
    print("\n[SYSTEM] Booting Extrinsic Calibration Module (ArUco Method)...")

    try:
        master = mavutil.mavlink_connection(PORT, baud=BAUD_RATE)
        master.wait_heartbeat(timeout=5)
        # Request high-frequency attitude data
        master.mav.request_data_stream_send(master.target_system, master.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 50, 1)
        print(f"[SUCCESS] Flight Controller Linked on {PORT}.")
    except Exception as e:
        print(f"[ERROR] Flight Controller connection failed: {e}")
        return None, None, None

    # Hardware connection: Try external USB first (1), fallback to built-in (0)
    cap = cv2.VideoCapture(1) 
    if not cap.isOpened(): cap = cv2.VideoCapture(0)
    
    # 1. Define the 3D points of the physical marker
    obj_points = np.array([
        [-MARKER_SIZE/2,  MARKER_SIZE/2, 0.0], 
        [ MARKER_SIZE/2,  MARKER_SIZE/2, 0.0], 
        [ MARKER_SIZE/2, -MARKER_SIZE/2, 0.0], 
        [-MARKER_SIZE/2, -MARKER_SIZE/2, 0.0]  
    ], dtype=np.float32)

    # 2. Camera Matrix (Intrinsic)
    camera_matrix = np.array([
        [FOCAL_LENGTH, 0, CENTER_X],
        [0, FOCAL_LENGTH, CENTER_Y],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros((4, 1))

    # 3. ArUco Setup
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)

    print(f"\n[READY] Point Camera at the ArUco Marker.")
    print(f"[SYSTEM] Auto-Locking after {REQUIRED_SAMPLES} valid frames...\n")

    pitch_offsets = []
    roll_offsets = []
    valid_samples = 0
    last_print_time = time.time()

    while valid_samples < REQUIRED_SAMPLES:
        ret, frame = cap.read()
        if not ret: break

        imu_pitch = imu_roll = 0.0
        msg = master.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            imu_pitch = math.degrees(msg.pitch)
            imu_roll  = math.degrees(msg.roll)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            # Solve Perspective-n-Point
            success, rotation_vector, translation_vector = cv2.solvePnP(
                obj_points, corners[0], camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if success:
                R_matrix, _ = cv2.Rodrigues(rotation_vector)
                # Extract Euler Angles matching MAVLink convention
                euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(R_matrix)
                
                cam_pitch_raw = euler_angles[0]
                cam_roll_raw  = euler_angles[2]

                # Custom axis transformation
                cam_pitch = abs(cam_pitch_raw) - 90.0
                cam_roll  = cam_roll_raw

                # Calculate relative offset
                offset_pitch = imu_pitch - cam_pitch
                offset_roll  = imu_roll - cam_roll

                pitch_offsets.append(offset_pitch)
                roll_offsets.append(offset_roll)

                valid_samples += 1
                print(f"\r[LOCKING] Sample {valid_samples}/{REQUIRED_SAMPLES} -> Pitch Offset: {offset_pitch:.2f} | Roll Offset: {offset_roll:.2f}      ", end="")
        else:
            if time.time() - last_print_time > 1.0:
                print("\rSearching for marker...                                 ", end="")
                last_print_time = time.time()

    cap.release()

    if len(pitch_offsets) > 10:
        raw_pitch = np.median(pitch_offsets)
        raw_roll = np.median(roll_offsets)
        
        # Apply Deadband Filter
        final_pitch = snap_to_zero(raw_pitch)
        final_roll = snap_to_zero(raw_roll)
        final_yaw = 0.0  # Yaw is computationally ignored for extrinsic mounting safely
        
        print(f"\n\n[SUCCESS] Locked & Cleaned Angles -> Pitch: {final_pitch}, Roll: {final_roll}, Yaw: {final_yaw}")
        master.close()
        
        return final_roll, final_pitch, final_yaw
    else:
        print("\n[ERROR] Calibration failed. Marker not seen.")
        master.close()
        return None, None, None

if __name__ == "__main__":
    run_calibration()