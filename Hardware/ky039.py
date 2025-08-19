from machine import ADC, Pin
import time

# Configure the analog pin for the KY-039 sensor
adc = ADC(Pin(3))
adc.atten(ADC.ATTN_11DB)  # Full range: 3.3V
adc.width(ADC.WIDTH_12BIT)  # 12-bit resolution (0-4095)

def calculate_heart_rate():
    # Variables to track heart rate calculation
    beats = []
    beat_times = []
    last_beat_time = 0
    measuring = False
    
    # Buffers for readings
    readings_buffer = []
    
    print("Place your finger on the sensor...")
    time.sleep(3)
    
    # Calibration phase with improved detection
    print("Calibrating (10 seconds)...")
    calibration_start = time.ticks_ms()
    
    while time.ticks_diff(time.ticks_ms(), calibration_start) < 10000:
        value = adc.read()
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
    threshold = avg_reading + std_dev * 0.5
    
    print(f"Calibrated - Avg: {avg_reading:.1f}, StdDev: {std_dev:.1f}, Threshold: {threshold:.1f}")
    
    # Clear buffer for measurement phase
    readings_buffer = []
    smoothed_values = []
    
    # Start actual heart rate measurement
    print("Measuring heart rate for 20 seconds...")
    print("Keep finger still for accurate reading")
    start_time = time.ticks_ms()
    last_print_time = start_time
    
    while time.ticks_diff(time.ticks_ms(), start_time) < 20000:
        current_time = time.ticks_ms()
        value = adc.read()
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
            print(f"Value: {smoothed:.0f}, Threshold: {threshold:.0f}")
        
        # Peak detection with hysteresis
        if len(smoothed_values) > 3:
            # Get the most recent values
            recent_values = smoothed_values[-3:]
            
            # Check if middle value is a peak (higher than neighbors)
            if recent_values[1] > recent_values[0] and recent_values[1] > recent_values[2] and recent_values[1] > threshold:
                # We found a peak, likely a heartbeat
                if measuring and last_beat_time > 0:
                    beat_interval = time.ticks_diff(current_time, last_beat_time)
                    
                    # Only accept reasonable intervals (30-180 BPM)
                    if 333 < beat_interval < 2000:
                        beats.append(beat_interval)
                        beat_times.append(current_time)
                        instant_bpm = 60000 / beat_interval
                        print(f"Beat detected - Instant BPM: {instant_bpm:.1f}")
                
                last_beat_time = current_time
                measuring = True
                
                # After detecting a beat, wait a bit to avoid double-counting
                time.sleep_ms(200)
        
        time.sleep_ms(10)
    
    # Calculate final heart rate using various methods
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
        
        return bpm, len(beats) + 1
    else:
        return 0, 0

# Main loop
try:
    while True:
        bpm, beats_detected = calculate_heart_rate()
        
        if bpm > 0:
            print(f"\nMeasurement complete:")
            print(f"Heart Rate: {bpm:.1f} BPM")
            print(f"Beats detected: {beats_detected}")
            
            # Interpret the heart rate (optional)
            if bpm < 60:
                print("This is below typical resting heart rate (bradycardia).")
            elif bpm > 100:
                print("This is above typical resting heart rate (tachycardia).")
            else:
                print("This is within normal resting heart rate range.")
        else:
            print("\nNot enough heartbeats detected. Please try again.")
            print("Tips: Make sure your finger is placed firmly on the sensor.")
            print("      Try to keep your finger still during measurement.")
        
        print("\nWait 5 seconds before next reading...")
        time.sleep(5)
        
except KeyboardInterrupt:
    print("Measurement stopped by user")