from imu import MPU6050
from time import sleep
from machine import Pin, I2C
import math

# Initialize I2C communication
i2c = I2C(0, sda=Pin(13), scl=Pin(14), freq=400000)
imu = MPU6050(i2c)

# Set threshold values for fall detection
acceleration_threshold = 1.5 * 9.81  # 1.5g in m/s²
gyroscope_threshold = 30  # 30 degrees per second

while True:
    # Read accelerometer and gyroscope data
    ax = round(imu.accel.x, 2)
    ay = round(imu.accel.y, 2)
    az = round(imu.accel.z, 2)
    gx = round(imu.gyro.x)
    gy = round(imu.gyro.y)
    gz = round(imu.gyro.z)
    tem = round(imu.temperature, 2)

    # Print sensor data
    print(f"ax: {ax}\tay: {ay}\taz: {az}\tgx: {gx}\tgy: {gy}\tgz: {gz}\tTemperature: {tem}", end="\r")

    # Calculate the magnitude of acceleration and gyroscope readings correctly
    acceleration_magnitude = math.sqrt(ax**2 + ay**2 + az**2)
    gyroscope_magnitude = math.sqrt(gx**2 + gy**2 + gz**2)

    # Check for fall detection
    if acceleration_magnitude > acceleration_threshold and gyroscope_magnitude > gyroscope_threshold:
        print("\nFall detected! Stopping sensor readings.")
        break  # Stop the loop when a fall is detected

    # Delay before the next reading
    sleep(0.2)
