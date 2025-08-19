from imu import MPU6050
from time import sleep
from machine import Pin, I2C
from neopixel import NeoPixel
import math
import sys

class FallDetector:
    def __init__(self):
        # Initialize I2C and MPU6050
        self.i2c = I2C(0, sda=Pin(13), scl=Pin(14), freq=400000)
        self.imu = MPU6050(self.i2c)
        
        # Modified thresholds for better fall detection
        self.ACCEL_THRESHOLD = 1.8     # Reduced from 2.5 to detect less intense falls
        self.GYRO_THRESHOLD = 150      # Reduced from 250 for more sensitive rotation detection
        self.STABLE_THRESHOLD = 0.5    # Reduced from 0.8 to detect more subtle stable positions
        
        # Built-in LED for visual indication
        self.led = Pin(2, Pin.OUT)
        
        # Initialize NeoPixel for emergency light
        self.np_pin = Pin(48, Pin.OUT)  # Pin number for v1 of the DevKitC, use pin 38 for v1.1
        self.np = NeoPixel(self.np_pin, 1)  # "1" = one RGB LED on the "led bus"
        
        self.fall_detected = False
        
    def calculate_magnitude(self, x, y, z):
        return math.sqrt(x*x + y*y + z*z)
    
    def emergency_blink(self, duration=10):
        """Blink red and blue lights for a specified duration in seconds"""
        end_time = time.time() + duration
        
        while time.time() < end_time:
            # Red light
            self.np[0] = (255, 0, 0)
            self.np.write()
            sleep(0.5)
            
            # Blue light
            self.np[0] = (0, 0, 255)
            self.np.write()
            sleep(0.5)
        
        # Turn off LED after blinking
        self.np[0] = (0, 0, 0)
        self.np.write()
    
    def check_fall(self):
        try:
            # Read sensor data
            ax = round(self.imu.accel.x, 2)
            ay = round(self.imu.accel.y, 2)
            az = round(self.imu.accel.z, 2)
            gx = abs(round(self.imu.gyro.x))
            gy = abs(round(self.imu.gyro.y))
            gz = abs(round(self.imu.gyro.z))
            
            # Calculate total acceleration and angular velocity
            total_accel = self.calculate_magnitude(ax, ay, az)
            total_gyro = max(gx, gy, gz)
            
            # Print current values
            print(f"Accel: {total_accel:.2f}g  Gyro: {total_gyro}°/s  ", end="\r")
            
            # Fall detection logic
            if total_accel > self.ACCEL_THRESHOLD and total_gyro > self.GYRO_THRESHOLD:
                # Wait a short moment to confirm the fall
                sleep(0.1)
                
                # Get new readings to confirm stable position
                ax_new = round(self.imu.accel.x, 2)
                ay_new = round(self.imu.accel.y, 2)
                az_new = round(self.imu.accel.z, 2)
                total_accel_new = self.calculate_magnitude(ax_new, ay_new, az_new)
                
                # Check if position is stable
                if abs(total_accel_new - 1) < self.STABLE_THRESHOLD:
                    print("\n\nFALL DETECTED!")
                    print(f"Final Acceleration: {total_accel_new:.2f}g")
                    print(f"Final Angular Velocity: {total_gyro}°/s")
                    self.led.on()
                    self.fall_detected = True
                    return True
                    
            return False
            
        except Exception as e:
            print(f"Error: {e}")
            return False

# Import time module for the emergency_blink function
import time

# Create fall detector instance
detector = FallDetector()

print("Fall detection started. Monitoring movements...")
print("Press Ctrl+C to stop the program")

try:
    while True:
        if detector.check_fall():
            print("Emergency lights activated!")
            detector.emergency_blink(10)  # Blink for 10 seconds
            print("\nProgram stopped due to fall detection.")
            sys.exit()
        sleep(0.2)
        
except KeyboardInterrupt:
    print("\nProgram stopped by user")
    sys.exit()