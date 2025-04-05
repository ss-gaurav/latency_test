#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

# Use BOARD numbering for the physical pin layout.
GPIO.setmode(GPIO.BOARD)

# For the Jetson Orin Nano, physical pin 32 corresponds to gpio07.
gpio_pin = 32

# Configure the pin as an output and initialize it to LOW.
GPIO.setup(gpio_pin, GPIO.OUT, initial=GPIO.LOW)

try:
    while True:
        # Set the pin HIGH (toggle ON).
        GPIO.output(gpio_pin, GPIO.HIGH)
        time.sleep(1)  # Wait for 1 second.
        
        # Set the pin LOW (toggle OFF).
        GPIO.output(gpio_pin, GPIO.LOW)
        time.sleep(1)  # Wait for 1 second.
except KeyboardInterrupt:
    print("Exiting the program.")
finally:
    # Reset the GPIO pins to a safe state.
    GPIO.cleanup()

