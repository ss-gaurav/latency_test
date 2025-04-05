#!/usr/bin/env python3
import cv2
import time
import numpy as np
import threading
import Jetson.GPIO as GPIO
from ultralytics import YOLO

# -------------------------
# Camera Loader Definition
# -------------------------
class CameraLoader:
    def __init__(self, device='/dev/video0'):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            print("Error: Could not open video stream from", device)
            exit()
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
    
    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame
            else:
                time.sleep(0.005)
    
    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
    
    def stop(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()

# -------------------------
# GPIO and LED Setup
# -------------------------
GPIO.setmode(GPIO.BOARD)
gpio_pin = 32  # Physical pin 32 corresponds to gpio07
GPIO.setup(gpio_pin, GPIO.OUT, initial=GPIO.LOW)

# -------------------------
# Initialize Camera Loader and YOLO Model
# -------------------------
camera_loader = CameraLoader('/dev/video0')
model = YOLO("yolov8n.pt")  # Adjust model path if needed

region_size = 20  # Define region size for center brightness check

# -------------------------
# Main Loop: LED Toggle and Latency Measurement
# -------------------------
try:
    while True:
        # Turn LED ON and record the timestamp.
        GPIO.output(gpio_pin, GPIO.HIGH)
        led_on_time = time.time()
        print(f"LED turned ON at {led_on_time:.4f}")

        # Wait until detection: check center region brightness of the latest frame.
        detection_done = False
        while not detection_done:
            frame = camera_loader.read()
            if frame is None:
                time.sleep(0.005)
                continue

            # Optionally, run YOLO inference if needed (commented out to save latency)
            # _ = model(frame)

            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            x1 = max(center_x - region_size // 2, 0)
            y1 = max(center_y - region_size // 2, 0)
            x2 = min(center_x + region_size // 2, w)
            y2 = min(center_y + region_size // 2, h)
            center_region = frame[y1:y2, x1:x2]
            
            # Compute the average brightness in the center region.
            gray = cv2.cvtColor(center_region, cv2.COLOR_BGR2GRAY)
            mean_intensity = np.mean(gray)
            
            if mean_intensity > 200:  # Adjust threshold as needed.
                detection_time = time.time()
                latency = detection_time - led_on_time
                print(f"LED detected at {detection_time:.4f} (Latency: {latency:.4f} seconds)")
                detection_done = True
            else:
                time.sleep(0.005)  # Prevent a tight loop.

        # Turn LED OFF after detection.
        GPIO.output(gpio_pin, GPIO.LOW)
        print("LED turned OFF.")

        # Wait until the center region becomes dark again before starting the next cycle.
        while True:
            frame = camera_loader.read()
            if frame is None:
                time.sleep(0.005)
                continue

            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2
            x1 = max(center_x - region_size // 2, 0)
            y1 = max(center_y - region_size // 2, 0)
            x2 = min(center_x + region_size // 2, w)
            y2 = min(center_y + region_size // 2, h)
            center_region = frame[y1:y2, x1:x2]
            gray = cv2.cvtColor(center_region, cv2.COLOR_BGR2GRAY)
            mean_intensity = np.mean(gray)
            
            if mean_intensity < 50:  # Dark threshold; adjust as needed.
                break
            time.sleep(0.005)
        
        # Optional delay before the next cycle.
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting program.")

finally:
    camera_loader.stop()
    GPIO.cleanup()

