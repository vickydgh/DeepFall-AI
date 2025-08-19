# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

# boot.py - runs on ESP32 boot to start main_ai.py

# boot.py - runs on ESP32 boot to start main_ai.py

from wifi import connect_wifi, WIFI_SSID, WIFI_PASSWORD
import main_ai

def main():
    if connect_wifi(WIFI_SSID, WIFI_PASSWORD):
        fall_system = main_ai.FallDetectionSystem()
        fall_system.run()
    else:
        print("WiFi connection failed. Cannot start fall detection system.")

if __name__ == "__main__":
    main()