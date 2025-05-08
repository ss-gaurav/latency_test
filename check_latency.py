#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
import cv2
import numpy as np
import threading

# ------------------ GPIO SETUP ------------------
GPIO.setmode(GPIO.BOARD)
gpio_pin = 32
GPIO.setup(gpio_pin, GPIO.OUT, initial=GPIO.LOW)

# Shared variable for GPIO toggle timestamp
toggle_timestamp = None
lock = threading.Lock()

# ------------------ GPIO TOGGLE THREAD ------------------
def gpio_toggle():
    global toggle_timestamp
    while True:
        GPIO.output(gpio_pin, GPIO.HIGH)
        with lock:
            toggle_timestamp = time.time()
        time.sleep(0.5)

        GPIO.output(gpio_pin, GPIO.LOW)
        time.sleep(0.5)

# Start GPIO toggle in background
gpio_thread = threading.Thread(target=gpio_toggle, daemon=True)
gpio_thread.start()

# ------------------ CAMERA SETUP ------------------
pipeline = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=3840, height=2160, framerate=30/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! "
    "appsink drop=true sync=false max-buffers=1"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open camera.")
    GPIO.cleanup()
    exit()

print("Camera started. Press ESC to exit.")

# Define ROI
x, y, w, h = 1900, 1000, 10, 10
first_time = True

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab frame.")
            break

        roi = frame[y:y+h, x:x+w]
        mean_blue = cv2.mean(roi)[0]

        # Visual indicator
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # Detection logic with latency calculation
        if mean_blue < 200:
            first_time = True

        if mean_blue > 200 and first_time:
            with lock:
                if toggle_timestamp is not None:
                    latency = time.time() - toggle_timestamp
                    print(f"Latency: {latency * 1000:.2f} ms")
            first_time = False

        #cv2.imshow("Camera", frame)
        #if cv2.waitKey(1) == 27:  # ESC key
            #break

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

