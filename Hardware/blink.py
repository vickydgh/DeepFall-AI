import _thread
import time
from machine import Pin
from neopixel import NeoPixel
import math

pin = Pin(48, Pin.OUT)      # Pin number for v1 of the DevKitC, use pin 38 for v1.1
np = NeoPixel(pin, 1)       # "1" = one RGB LED on the "led bus"

# Thread control
_animation_thread_running = False

def stop_animation():
    global _animation_thread_running
    _animation_thread_running = False
    time.sleep(0.1)  # Give time for thread to stop

def start_animation(target_func, *args):
    global _animation_thread_running
    stop_animation()
    _animation_thread_running = True
    _thread.start_new_thread(target_func, args)

def _blink(color, interval, times=None):
    count = 0
    while _animation_thread_running and (times is None or count < times):
        np[0] = color
        np.write()
        time.sleep(interval)
        np[0] = (0, 0, 0)
        np.write()
        time.sleep(interval)
        count += 1

def _pulse(color):
    while _animation_thread_running:
        for i in range(0, 256, 5):
            if not _animation_thread_running:
                break
            scaled_color = tuple(int(c * i / 255) for c in color)
            np[0] = scaled_color
            np.write()
            time.sleep(0.02)
        for i in range(255, -1, -5):
            if not _animation_thread_running:
                break
            scaled_color = tuple(int(c * i / 255) for c in color)
            np[0] = scaled_color
            np.write()
            time.sleep(0.02)

def _steady(color):
    while _animation_thread_running:
        np[0] = color
        np.write()
        time.sleep(0.1)

def _blink_twice(color, interval):
    while _animation_thread_running:
        for _ in range(2):
            if not _animation_thread_running:
                break
            np[0] = color
            np.write()
            time.sleep(interval)
            np[0] = (0, 0, 0)
            np.write()
            time.sleep(interval)
        time.sleep(2)  # Off for 2 seconds

def _rapid_alternating(color1, color2, interval):
    while _animation_thread_running:
        np[0] = color1
        np.write()
        time.sleep(interval)
        if not _animation_thread_running:
            break
        np[0] = color2
        np.write()
        time.sleep(interval)

def _fade(color):
    while _animation_thread_running:
        for i in range(0, 256, 5):
            if not _animation_thread_running:
                break
            scaled_color = tuple(int(c * i / 255) for c in color)
            np[0] = scaled_color
            np.write()
            time.sleep(0.02)
        for i in range(255, -1, -5):
            if not _animation_thread_running:
                break
            scaled_color = tuple(int(c * i / 255) for c in color)
            np[0] = scaled_color
            np.write()
            time.sleep(0.02)

# Public functions to start animations for each system state

def power_on():
    """Blink white slowly (1s ON/OFF)"""
    start_animation(_blink, (255, 255, 255), 1)

def heart_rate_calibration():
    """Blue pulsing (fade in and out)"""
    start_animation(_pulse, (0, 0, 255))

def heart_rate_reading():
    """Steady blue"""
    start_animation(_steady, (0, 0, 255))

def fall_detection_active():
    """Blink green twice quickly then off for 2 seconds cycle"""
    start_animation(_blink_twice, (0, 255, 0), 0.2)

def fall_detected():
    """Rapid alternating red and blue every 300ms"""
    start_animation(_rapid_alternating, (255, 0, 0), (0, 0, 255), 0.3)

def system_reset():
    """Fade between yellow and off over 2 seconds"""
    start_animation(_fade, (255, 255, 0))

def sensor_error():
    """Steady purple"""
    start_animation(_steady, (128, 0, 128))

def emergency_blink(duration=10):
    """Blink red and blue lights to indicate emergency for duration seconds"""
    stop_animation()
    end_time = time.time() + duration
    while time.time() < end_time:
        np[0] = (255, 0, 0)
        np.write()
        time.sleep(0.5)
        np[0] = (0, 0, 255)
        np.write()
        time.sleep(0.5)
    np[0] = (0, 0, 0)
    np.write()
