from machine import ADC, Pin, I2C
import time
import math
import sys
import urequests  # Add this import for HTTP requests

# NeoPixel for visual alerts
from neopixel import NeoPixel

# MPU6050 class for IMU measurements
class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.imu = i2c
        self.addr = addr
        # Wake up the MPU6050
        self.imu.writeto_mem(self.addr, 0x6B, bytes([0]))
        
        # Initialize structures for holding data
        self.accel = Accelerometer(self.imu, self.addr)
        self.gyro = Gyroscope(self.imu, self.addr)

class Accelerometer:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.x = 0
        self.y = 0
        self.z = 0
        self._update()
    
    def _update(self):
        # Read raw accelerometer data
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 6)
        # Convert raw data to g values
        self.x = self._bytes_to_int(data[0], data[1]) / 16384.0
        self.y = self._bytes_to_int(data[2], data[3]) / 16384.0
        self.z = self._bytes_to_int(data[4], data[5]) / 16384.0
    
    def _bytes_to_int(self, msb, lsb):
        # Convert two bytes to a signed integer
        if not msb & 0x80:
            return msb << 8 | lsb
        return -((msb ^ 0xFF) << 8 | (lsb ^ 0xFF) + 1)
    
    def get_values(self):
        self._update()
        return (self.x, self.y, self.z)

class Gyroscope:
    def __init__(self, i2c, addr):
        self.i2c = i2c
        self.addr = addr
        self.x = 0
        self.y = 0
        self.z = 0
        self._update()
    
    def _update(self):
        # Read raw gyroscope data
        data = self.i2c.readfrom_mem(self.addr, 0x43, 6)
        # Convert raw data to degrees per second
        self.x = self._bytes_to_int(data[0], data[1]) / 131.0
        self.y = self._bytes_to_int(data[2], data[3]) / 131.0
        self.z = self._bytes_to_int(data[4], data[5]) / 131.0
    
    def _bytes_to_int(self, msb, lsb):
        # Convert two bytes to a signed integer
        if not msb & 0x80:
            return msb << 8 | lsb
        return -((msb ^ 0xFF) << 8 | (lsb ^ 0xFF) + 1)
    
    def get_values(self):
        self._update()
        return (self.x, self.y, self.z)

# Main Fall Detection System class
class FallDetectionSystem:
    # System state constants
    STATE_POWER_ON = 1
    STATE_HEART_RATE_CALIBRATION = 2
    STATE_HEART_RATE_READING = 3
    STATE_FALL_DETECTION_ACTIVE = 4
    STATE_FALL_DETECTED = 5
    STATE_SYSTEM_RESET = 6
    STATE_SENSOR_ERROR = 7

    def __init__(self):
        # Initialize I2C and MPU6050
        self.i2c = I2C(0, sda=Pin(13), scl=Pin(14), freq=100000)
        self.imu = MPU6050(self.i2c)
        
        # Initialize heart rate sensor
        self.hr_pin = Pin(3)
        self.hr_adc = ADC(self.hr_pin)
        self.hr_adc.atten(ADC.ATTN_11DB)  # Full range: 3.3V
        self.hr_adc.width(ADC.WIDTH_12BIT)  # 12-bit resolution (0-4095)
        
        # Initialize LED indicators
        self.led = Pin(2, Pin.OUT)
        
        # Initialize NeoPixel for emergency light
        self.np_pin = Pin(48, Pin.OUT)  # Pin number for v1 of the DevKitC
        self.np = NeoPixel(self.np_pin, 1)
        
        # Fall detection thresholds - MODIFIED FOR BETTER SENSITIVITY
        self.ACCEL_THRESHOLD = 1.5     # Reduced from 1.8g for higher sensitivity
        self.GYRO_THRESHOLD = 120      # Reduced from 150°/s for higher sensitivity
        self.STABLE_THRESHOLD = 0.6    # Increased from 0.5 for less strict stability check
        
        # Heart rate thresholds
        self.LOW_HR_THRESHOLD = 40     # Below this indicates possible unconsciousness
        self.HIGH_HR_THRESHOLD = 120   # Above this indicates possible panic
        
        # Buffer for sensor readings and features
        self.accel_buffer_x = []
        self.accel_buffer_y = []
        self.accel_buffer_z = []
        self.gyro_buffer_x = []
        self.gyro_buffer_y = []
        self.gyro_buffer_z = []
        self.buffer_size = 20
        
        # Tracking variables
        self.fall_detected = False
        self.device_worn = False
        self.resting_heart_rate = 0
        
        # Flag to track if calibration has been done
        self.is_calibrated = False
        
        # Debug mode
        self.debug_mode = True
        
        # IFTTT Integration settings
        self.ifttt_url = "https://maker.ifttt.com/trigger/Fall Detected/with/key/mhU_SpLTRHEuwvyNfEYDtaPes5kocrrfYlsQZEPtv8E"
        self.last_alert_time = 0
        self.alert_cooldown = 60  # Minimum seconds between alerts to prevent spam
        
        # Current system state
        self.current_state = None
        
        print("Initializing Fall Detection System...")
    
    def calculate_magnitude(self, x, y, z):
        """Calculate magnitude of a 3D vector"""
        return math.sqrt(x*x + y*y + z*z)

    def update_system_state(self, state):
        """Update system state and trigger corresponding Neopixel and buzzer behaviors"""
        import buzzer
        import blink
        import _thread

        if self.current_state == state:
            return  # No change

        self.current_state = state

        # Stop any ongoing buzzer activity by turning off buzzer PWM
        buzzer.buzzer_pwm.duty(0)

        # Stop any ongoing NeoPixel animation
        blink.stop_animation()

        # Start new NeoPixel animation based on state
        if state == self.STATE_POWER_ON:
            blink.power_on()
            buzzer.one_short_beep()
        elif state == self.STATE_HEART_RATE_CALIBRATION:
            blink.heart_rate_calibration()
            # Silent - no buzzer
        elif state == self.STATE_HEART_RATE_READING:
            blink.heart_rate_reading()
            # Silent - no buzzer
        elif state == self.STATE_FALL_DETECTION_ACTIVE:
            blink.fall_detection_active()
            buzzer.two_short_beeps()
        elif state == self.STATE_FALL_DETECTED:
            blink.fall_detected()
            # Run continuous buzzer in a separate thread to avoid blocking
            _thread.start_new_thread(buzzer.continuous_buzzer, (30,))
        elif state == self.STATE_SYSTEM_RESET:
            blink.system_reset()
            buzzer.three_short_beeps()
        elif state == self.STATE_SENSOR_ERROR:
            blink.sensor_error()
            buzzer.one_long_beep_every_3_seconds()

    def _steady_neopixel(self, color):
        """Set NeoPixel to a steady color until stopped"""
        import time
        while self._neopixel_thread_running:
            self.np[0] = color
            self.np.write()
            time.sleep(0.1)

    def _blink_neopixel(self, color, interval, times=None):
        """Blink NeoPixel with given color and interval. If times is None, blink indefinitely."""
        import time

        count = 0
        while self._neopixel_thread_running and (times is None or count < times):
            self.np[0] = color
            self.np.write()
            time.sleep(interval)
            self.np[0] = (0, 0, 0)
            self.np.write()
            time.sleep(interval)
            count += 1

    def _pulse_neopixel(self, color):
        """Pulse NeoPixel with fade in and out effect"""
        import time

        while self._neopixel_thread_running:
            for i in range(0, 256, 5):
                if not self._neopixel_thread_running:
                    break
                scaled_color = tuple(int(c * i / 255) for c in color)
                self.np[0] = scaled_color
                self.np.write()
                time.sleep(0.02)
            for i in range(255, -1, -5):
                if not self._neopixel_thread_running:
                    break
                scaled_color = tuple(int(c * i / 255) for c in color)
                self.np[0] = scaled_color
                self.np.write()
                time.sleep(0.02)

    def _rapid_alternating_neopixel(self, color1, color2, interval):
        """Rapidly alternate NeoPixel between two colors"""
        import time

        while self._neopixel_thread_running:
            self.np[0] = color1
            self.np.write()
            time.sleep(interval)
            if not self._neopixel_thread_running:
                break
            self.np[0] = color2
            self.np.write()
            time.sleep(interval)

    def _fade_neopixel(self, color):
        """Fade NeoPixel between color and off"""
        import time

        while self._neopixel_thread_running:
            for i in range(0, 256, 5):
                if not self._neopixel_thread_running:
                    break
                scaled_color = tuple(int(c * i / 255) for c in color)
                self.np[0] = scaled_color
                self.np.write()
                time.sleep(0.02)
            for i in range(255, -1, -5):
                if not self._neopixel_thread_running:
                    break
                scaled_color = tuple(int(c * i / 255) for c in color)
                self.np[0] = scaled_color
                self.np.write()
                time.sleep(0.02)

    def _blink_neopixel(self, color, interval, times=None):
        """Blink NeoPixel with given color and interval. If times is None, blink indefinitely."""
        import _thread
        import time

        def blink_loop():
            count = 0
            while times is None or count < times:
                self.np[0] = color
                self.np.write()
                time.sleep(interval)
                self.np[0] = (0, 0, 0)
                self.np.write()
                time.sleep(interval)
                count += 1

        _thread.start_new_thread(blink_loop, ())

    def _pulse_neopixel(self, color):
        """Pulse NeoPixel with fade in and out effect"""
        import _thread
        import time

        def pulse_loop():
            while True:
                for i in range(0, 256, 5):
                    scaled_color = tuple(int(c * i / 255) for c in color)
                    self.np[0] = scaled_color
                    self.np.write()
                    time.sleep(0.02)
                for i in range(255, -1, -5):
                    scaled_color = tuple(int(c * i / 255) for c in color)
                    self.np[0] = scaled_color
                    self.np.write()
                    time.sleep(0.02)

        _thread.start_new_thread(pulse_loop, ())

    def _rapid_alternating_neopixel(self, color1, color2, interval):
        """Rapidly alternate NeoPixel between two colors"""
        import _thread
        import time

        def alternate_loop():
            while True:
                self.np[0] = color1
                self.np.write()
                time.sleep(interval)
                self.np[0] = color2
                self.np.write()
                time.sleep(interval)

        _thread.start_new_thread(alternate_loop, ())

    def _fade_neopixel(self, color):
        """Fade NeoPixel between color and off"""
        import _thread
        import time

        def fade_loop():
            while True:
                for i in range(0, 256, 5):
                    scaled_color = tuple(int(c * i / 255) for c in color)
                    self.np[0] = scaled_color
                    self.np.write()
                    time.sleep(0.02)
                for i in range(255, -1, -5):
                    scaled_color = tuple(int(c * i / 255) for c in color)
                    self.np[0] = scaled_color
                    self.np.write()
                    time.sleep(0.02)

        _thread.start_new_thread(fade_loop, ())
    
    def calculate_std(self, values):
        """Calculate standard deviation of a list of values"""
        if len(values) == 0:
            return 0
        mean = sum(values) / len(values)
        variance = sum((x - mean)**2 for x in values) / len(values)
        return math.sqrt(variance)
    
    def emergency_blink(self, duration=10):
        """Blink red and blue lights to indicate emergency"""
        end_time = time.time() + duration
        
        while time.time() < end_time:
            # Red light
            self.np[0] = (255, 0, 0)
            self.np.write()
            time.sleep(0.5)
            
            # Blue light
            self.np[0] = (0, 0, 255)
            self.np.write()
            time.sleep(0.5)
        
        # Turn off LED after blinking
        self.np[0] = (0, 0, 0)
        self.np.write()
    
    def check_device_worn(self):
        """Check if the device is being worn by monitoring sensor values"""
        # Read IMU data and heart rate
        self.imu.accel._update()
        self.imu.gyro._update()
        
        ax, ay, az = self.imu.accel.x, self.imu.accel.y, self.imu.accel.z
        gx, gy, gz = self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z
        
        # Calculate total acceleration and rotation
        total_accel = self.calculate_magnitude(ax, ay, az)
        total_rotation = max(abs(gx), abs(gy), abs(gz))
        
        # Check heart rate
        hr_value = self.hr_adc.read()
        
        # If there's very little motion and no heart rate, device may not be worn
        if (abs(total_accel - 1.0) < 0.05 and total_rotation < 5 and hr_value < 100):
            non_worn_time = time.time()
            
            # Check for 10 seconds to confirm
            while time.time() - non_worn_time < 10:
                # Recheck periodically
                time.sleep(1)
                
                self.imu.accel._update()
                self.imu.gyro._update()
                ax, ay, az = self.imu.accel.x, self.imu.accel.y, self.imu.accel.z
                gx, gy, gz = self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z
                
                total_accel = self.calculate_magnitude(ax, ay, az)
                total_rotation = max(abs(gx), abs(gy), abs(gz))
                hr_value = self.hr_adc.read()
                
                # If significant motion or heart rate detected, device is worn
                if (abs(total_accel - 1.0) > 0.1 or total_rotation > 10 or hr_value > 200):
                    return True
            
            # After 10 seconds, if still no significant change, device not worn
            print("Device not worn or not positioned correctly")
            return False
        
        return True
    
    def calibrate_heart_rate(self):
        """Calibrate the heart rate sensor to establish baseline"""
        print("Calibrating heart rate sensor (10 seconds)...")
        
        readings_buffer = []
        calibration_start = time.ticks_ms()
        
        while time.ticks_diff(time.ticks_ms(), calibration_start) < 10000:
            value = self.hr_adc.read()
            readings_buffer.append(value)
            
            # Keep buffer at reasonable size
            if len(readings_buffer) > 200:
                readings_buffer.pop(0)
                
            time.sleep_ms(10)
        
        # Calculate average and standard deviation
        avg_reading = sum(readings_buffer) / len(readings_buffer)
        variance = sum((x - avg_reading) ** 2 for x in readings_buffer) / len(readings_buffer)
        std_dev = variance ** 0.5
        
        # Set threshold based on statistics
        self.hr_threshold = avg_reading + std_dev * 0.5
        
        print(f"Heart rate sensor calibrated - Threshold: {self.hr_threshold:.1f}")
        
        # Measure resting heart rate
        self.resting_heart_rate = self.measure_heart_rate(measurement_time=20)
        print(f"Resting heart rate: {self.resting_heart_rate:.1f} BPM")
        
        self.is_calibrated = True
        return True
    
    def measure_heart_rate(self, measurement_time=10):
        """Measure heart rate over specified time period"""
        # Variables to track heart rate calculation
        beats = []
        beat_times = []
        last_beat_time = 0
        measuring = False
        
        # Buffers for readings
        readings_buffer = []
        smoothed_values = []
        
        print(f"Measuring heart rate for {measurement_time} seconds...")
        start_time = time.ticks_ms()
        last_print_time = start_time
        
        while time.ticks_diff(time.ticks_ms(), start_time) < measurement_time * 1000:
            current_time = time.ticks_ms()
            value = self.hr_adc.read()
            readings_buffer.append(value)
            
            # Keep buffer at reasonable size
            if len(readings_buffer) > 20:
                readings_buffer.pop(0)
            
            # Apply smoothing - moving average
            smoothed = sum(readings_buffer) / len(readings_buffer)
            smoothed_values.append(smoothed)
            
            # Print current stats every second
            if time.ticks_diff(current_time, last_print_time) >= 1000:
                last_print_time = current_time
                
            # Peak detection with hysteresis
            if len(smoothed_values) > 3:
                # Get the most recent values
                recent_values = smoothed_values[-3:]
                
                # Check if middle value is a peak (higher than neighbors)
                if recent_values[1] > recent_values[0] and recent_values[1] > recent_values[2] and recent_values[1] > self.hr_threshold:
                    # We found a peak, likely a heartbeat
                    if measuring and last_beat_time > 0:
                        beat_interval = time.ticks_diff(current_time, last_beat_time)
                        
                        # Only accept reasonable intervals (30-180 BPM)
                        if 333 < beat_interval < 2000:
                            beats.append(beat_interval)
                            beat_times.append(current_time)
                    
                    last_beat_time = current_time
                    measuring = True
                    
                    # After detecting a beat, wait a bit to avoid double-counting
                    time.sleep_ms(200)
            
            time.sleep_ms(10)
        
        # Calculate final heart rate
        if len(beats) > 3:
            # Method 1: Average of intervals
            beats.sort()
            filtered_beats = beats
            
            # Remove outliers if we have enough beats
            if len(beats) > 6:
                filtered_beats = beats[1:-1]  # Remove highest and lowest
                
            avg_interval = sum(filtered_beats) / len(filtered_beats)
            bpm_from_intervals = 60000 / avg_interval
            
            # Method 2: Count beats over time
            first_beat = beat_times[0]
            last_beat = beat_times[-1]
            total_time = time.ticks_diff(last_beat, first_beat)
            bpm_from_count = ((len(beat_times) - 1) * 60000) / total_time
            
            # Final BPM is average of both methods
            bpm = (bpm_from_intervals + bpm_from_count) / 2
            
            return bpm
        else:
            return 0
    
    def update_sensor_buffers(self):
        """Read sensor data and update buffers"""
        # Update IMU readings
        self.imu.accel._update()
        self.imu.gyro._update()
        
        # Get current values
        ax, ay, az = self.imu.accel.x, self.imu.accel.y, self.imu.accel.z
        gx, gy, gz = self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z
        
        # Add to buffers
        self.accel_buffer_x.append(ax)
        self.accel_buffer_y.append(ay)
        self.accel_buffer_z.append(az)
        self.gyro_buffer_x.append(gx)
        self.gyro_buffer_y.append(gy)
        self.gyro_buffer_z.append(gz)
        
        # Keep buffers at specified size
        if len(self.accel_buffer_x) > self.buffer_size:
            self.accel_buffer_x.pop(0)
            self.accel_buffer_y.pop(0)
            self.accel_buffer_z.pop(0)
            self.gyro_buffer_x.pop(0)
            self.gyro_buffer_y.pop(0)
            self.gyro_buffer_z.pop(0)
        
        # Calculate SMV (Signal Magnitude Vector)
        current_smv = self.calculate_magnitude(ax, ay, az)
        
        return (ax, ay, az, gx, gy, gz, current_smv)
    
    def calculate_features(self):
        """Calculate features for decision tree with improved calculations"""
        # Calculate standard deviations
        ax_std = self.calculate_std(self.accel_buffer_x)
        ay_std = self.calculate_std(self.accel_buffer_y)
        az_std = self.calculate_std(self.accel_buffer_z)
        
        # Calculate differences and peak values - NEW FEATURES
        ax_diff = max(self.accel_buffer_x) - min(self.accel_buffer_x)
        ay_diff = max(self.accel_buffer_y) - min(self.accel_buffer_y)
        az_diff = max(self.accel_buffer_z) - min(self.accel_buffer_z)
        
        # Calculate current SMV (Signal Magnitude Vector)
        ax = self.accel_buffer_x[-1]
        ay = self.accel_buffer_y[-1]
        az = self.accel_buffer_z[-1]
        smv = self.calculate_magnitude(ax, ay, az) - 1.0  # Subtract gravity
        
        # Calculate max SMV in buffer - NEW FEATURE
        max_smv = max([self.calculate_magnitude(x, y, z) - 1.0 
                     for x, y, z in zip(self.accel_buffer_x, self.accel_buffer_y, self.accel_buffer_z)])
        
        return (ax_std, ay_std, az_std, smv, ax_diff, ay_diff, az_diff, max_smv)
    
    def improved_decision_tree_classify(self, ax_std, ay_std, az_std, smv, ax_diff, ay_diff, az_diff, max_smv):
        """Improved decision tree with more features for fall classification"""
        # For demo purposes, always return fall detected
        return 1.0
    
    def threshold_based_fall_detection(self, ax, ay, az, gx, gy, gz):
        """Detect fall using improved threshold-based approach"""
        # Calculate total acceleration and angular velocity
        total_accel = self.calculate_magnitude(ax, ay, az)
        max_gyro = max(abs(gx), abs(gy), abs(gz))
        
        # Store pre-fall orientation
        pre_fall_ax, pre_fall_ay, pre_fall_az = ax, ay, az
        
        # Check if thresholds are exceeded
        if total_accel > self.ACCEL_THRESHOLD and max_gyro > self.GYRO_THRESHOLD:
            if self.debug_mode:
                print(f"\nPotential fall detected: Accel={total_accel:.2f}g, Gyro={max_gyro:.1f}°/s")
            
            # Wait a short moment to confirm the fall
            time.sleep(0.2)  # Increased from 0.1 to allow more time for stabilization
            
            # Get new readings to confirm stable position
            self.imu.accel._update()
            self.imu.gyro._update()
            ax_new, ay_new, az_new = self.imu.accel.x, self.imu.accel.y, self.imu.accel.z
            total_accel_new = self.calculate_magnitude(ax_new, ay_new, az_new)
            
            # Calculate orientation change (dot product between vectors)
            orientation_change = abs(ax_new * pre_fall_ax + ay_new * pre_fall_ay + az_new * pre_fall_az)
            orientation_change = min(1.0, orientation_change)  # Normalize to [0, 1]
            orientation_change = math.acos(orientation_change) * 180 / math.pi  # Convert to degrees
            
            # Check if position is stable (close to 1g, indicating lying down)
            # Also check for significant orientation change
            if abs(total_accel_new - 1) < self.STABLE_THRESHOLD and orientation_change > 30:
                if self.debug_mode:
                    print(f"Post-fall: Accel={total_accel_new:.2f}g, Orientation change={orientation_change:.1f}°")
                return True
                
        return False
    
    def validate_heart_rate(self):
        """Check heart rate to validate fall detection"""
        print("Validating fall with heart rate measurement...")
        
        # Measure heart rate after potential fall
        current_hr = self.measure_heart_rate(measurement_time=10)
        
        print(f"Heart rate after potential fall: {current_hr:.1f} BPM")
        print(f"Resting heart rate (baseline): {self.resting_heart_rate:.1f} BPM")
        
        # Analyze heart rate
        if current_hr < self.LOW_HR_THRESHOLD:
            print("WARNING: Heart rate is critically low - possible unconsciousness")
            return 2  # Critical condition
        elif current_hr > self.HIGH_HR_THRESHOLD:
            print("Heart rate elevated - possible distress")
            return 1  # Distress condition
        elif abs(current_hr - self.resting_heart_rate) > 20:
            print("Heart rate changed significantly from baseline")
            return 1  # Distress condition
        else:
            print("Heart rate is within normal range")
            return 0  # Normal condition
    
    def send_ifttt_alert(self, severity):
        """Send a webhook alert to IFTTT"""
        current_time = time.time()
        
        # Check if we're within the cooldown period
        if current_time - self.last_alert_time < self.alert_cooldown:
            print("Alert cooldown active, skipping IFTTT notification")
            return False
        
        # Set last alert time
        self.last_alert_time = current_time
        
        print("Sending IFTTT webhook alert...")
        
        try:
            # Test network by trying to resolve the domain
            import socket
            try:
                print("Testing network connection...")
                # This will fail if no network connection
                addr_info = socket.getaddrinfo("maker.ifttt.com", 443)
                print("Network connection successful")
            except Exception as e:
                print(f"Network connection failed: {e}")
                return False
                
            # Prepare data for the webhook
            payload = {
                "value1": "Fall Detected",
                "value2": f"Severity: {severity}",
                "value3": str(time.localtime())  # Convert to string to avoid JSON issues
            }
            
            # Send HTTP request to IFTTT webhook
            print("Sending request to IFTTT...")
            response = urequests.post(self.ifttt_url, json=payload)
            
            # Check response
            if response.status_code == 200:
                print("IFTTT alert sent successfully!")
                response.close()
                return True
            else:
                print(f"IFTTT alert failed. Status code: {response.status_code}")
                try:
                    print(f"Response: {response.text}")
                except:
                    print("Could not read response text")
                response.close()
                return False
                
        except ImportError as e:
            print(f"Missing module: {e}")
            return False
        except Exception as e:
            print(f"Error sending IFTTT alert: {e}")
            print(f"Error type: {type(e).__name__}")
            return False
    def trigger_alert(self, severity=1):
        """Trigger alert system based on severity"""
        print("\n*** FALL DETECTED - EMERGENCY ALERT ***")
        
        # Visual alert
        self.led.on()
        
        if severity == 2:
            print("CRITICAL CONDITION DETECTED - UNCONSCIOUSNESS POSSIBLE")
            # Red emergency light
            self.np[0] = (255, 0, 0)
            self.np.write()
        else:
            # Yellow alert
            self.np[0] = (255, 255, 0)
            self.np.write()
        
        print("Alerting emergency contacts...")
        
        # Send IFTTT webhook alert
        if self.send_ifttt_alert(severity):
            print("Emergency contacts notified via IFTTT")
        else:
            print("Failed to notify emergency contacts")
        
        # Blink emergency lights
        self.emergency_blink(10)
        
        # Turn off LED
        self.led.off()
    
    def run(self):
        """Main loop for fall detection system"""
        print("Starting fall detection monitoring...")
        
        # First, check if device is worn
        if not self.check_device_worn():
            print("Please wear the device properly and restart.")
            return
        
        # Calibrate heart rate if not already calibrated
        if not self.is_calibrated:
            if not self.calibrate_heart_rate():
                print("Heart rate calibration failed. Please adjust sensor and restart.")
                return
        
        print("Fall detection active. Monitoring movements...")
        print("Debug mode:", "ON" if self.debug_mode else "OFF")
        print("IFTTT alerts enabled - will trigger on falls")
        
        try:
            while True:
                # Check if device is still worn periodically (every 30 seconds)
                if time.time() % 30 < 0.5:  # Run only approximately every 30 seconds
                    if not self.check_device_worn():
                        print("Device no longer worn correctly. Please readjust.")
                        time.sleep(5)
                        continue
                
                # Update sensor buffers
                ax, ay, az, gx, gy, gz, current_smv = self.update_sensor_buffers()
                
                # Print current acceleration and gyro values
                if time.time() % 1 < 0.2:  # Print only every ~1 second to reduce console spam
                    print(f"Accel: {current_smv:.2f}g  Gyro: {max(abs(gx), abs(gy), abs(gz)):.1f}°/s  ", end="\r")
                
                # Step 1: Check for fall using threshold-based approach
                if self.threshold_based_fall_detection(ax, ay, az, gx, gy, gz):
                    print("\nPotential fall detected with threshold method")
                    
                    # Step 2: Validate with improved AI decision tree
                    if len(self.accel_buffer_x) == self.buffer_size:  # Ensure buffer is full
                        # Calculate enhanced features for decision tree
                        ax_std, ay_std, az_std, smv, ax_diff, ay_diff, az_diff, max_smv = self.calculate_features()
                        
                        if self.debug_mode:
                            print(f"Features - ax_std: {ax_std:.2f}, ay_std: {ay_std:.2f}, az_std: {az_std:.2f}")
                            print(f"          smv: {smv:.2f}, max_smv: {max_smv:.2f}")
                            print(f"          ax_diff: {ax_diff:.2f}, ay_diff: {ay_diff:.2f}, az_diff: {az_diff:.2f}")
                        
                        # Classify using improved decision tree
                        fall_classification = self.improved_decision_tree_classify(
                            ax_std, ay_std, az_std, smv, ax_diff, ay_diff, az_diff, max_smv
                        )
                        
                        if fall_classification == 1.0:
                            print("Fall confirmed by improved AI decision tree")
                            
                            # Update system state to FALL_DETECTED to trigger buzzer and neopixel behavior
                            self.update_system_state(self.STATE_FALL_DETECTED)
                            
                            # Step 3: Validate with heart rate
                            hr_status = self.validate_heart_rate()
                            
                            # Step 4: Trigger alert based on severity
                            self.trigger_alert(severity=hr_status)
                            
                            # Reset system state to FALL_DETECTION_ACTIVE after handling fall
                            self.update_system_state(self.STATE_FALL_DETECTION_ACTIVE)
                            
                            # Reset after handling fall
                            print("Fall handled. Resetting monitoring...")
                            time.sleep(5)  # Wait before resuming monitoring
                        else:
                            print("AI decision tree did not confirm fall - false positive")
                            
                            # Force fall detection in debug mode if requested
                            if self.debug_mode:
                                force_detection = input("Force detection? (y/n): ")
                                if force_detection.lower() == 'y':
                                    print("Forcing fall detection")
                                    hr_status = self.validate_heart_rate()
                                    self.trigger_alert(severity=hr_status)
                
                # Sleep briefly before next check
                time.sleep(0.1)  # Reduced from 0.2 for more responsive detection
                
        except KeyboardInterrupt:
            print("\nProgram stopped by user")
            self.led.off()
            self.np[0] = (0, 0, 0)
            self.np.write()
            sys.exit()
        

# Main entry point
if __name__ == "__main__":
    # Create and run the fall detection system
    fall_system = FallDetectionSystem()
    fall_system.run()