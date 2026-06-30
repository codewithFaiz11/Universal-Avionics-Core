"""
2D Camera Sensor Driver (ArUco / Blob Tracker)
Generalised for Universal Avionics Core
"""
import cv2
import numpy as np
import time

class SensorDriver:
    def __init__(self):
        print("[HARDWARE] Initializing 2D Vision Sensor...")
        self.CENTER_X = 320
        self.CENTER_Y = 240
        self.cap = None
        self.connect_hardware()

    def connect_hardware(self):
        """Attempts to bind to the USB port."""
        if self.cap is not None:
            self.cap.release()
            
        # Try camera 1 first (External USB), fallback to 0 (Built-in)
        self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        time.sleep(1.0) 

    def get_vector(self):
        """Returns the physical [X, Y, Z] array, or attempts to reconnect if failed."""
        # 1. Check if the object is dead
        if self.cap is None or not self.cap.isOpened():
            time.sleep(1.0) # Wait 1 second before retrying
            self.connect_hardware()
            return None

        # 2. Try to grab a frame
        ret, frame = self.cap.read() 
        
        # 3. If grabbing fails, the wire was just unplugged!
        if not ret: 
            self.cap.release() 
            self.cap = None    
            return None
            
        # 4. If healthy, do the math (Simple bright spot tracking)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (15, 15), 0)
        _, _, _, max_loc = cv2.minMaxLoc(gray)
        pixel_x, pixel_y = max_loc
        
        raw_x = (pixel_x - self.CENTER_X) * 0.01
        raw_y = (pixel_y - self.CENTER_Y) * 0.01
        raw_z = 0.0 # 2D camera doesn't provide depth directly here
        
        return np.array([raw_x, raw_y, raw_z]), None

    def close(self):
        if self.cap is not None:
            self.cap.release()
        print("[HARDWARE] Sensor safely disconnected.")