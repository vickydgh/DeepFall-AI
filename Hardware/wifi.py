import network
import time

def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(ssid, password)
        
        for i in range(20):
            if wlan.isconnected():
                break
            print(f"Attempt {i+1}/20... Connected: {wlan.isconnected()}")
            time.sleep(1)
    
    if wlan.isconnected():
        print("WiFi Connected!")
        print("Network config:", wlan.ifconfig())
        return True
    else:
        print("Connection failed.")
        return False

# Replace with your Wi-Fi credentials
WIFI_SSID = 'Galaxy F2257DC'
WIFI_PASSWORD = 'jmck9581'

connect_wifi(WIFI_SSID, WIFI_PASSWORD)
