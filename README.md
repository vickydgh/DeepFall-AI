# DeepFall AI: AI-Driven Heuristic Algorithms for IoT-Based Real-Time Fall Detection and Prevention  

This repository contains the implementation of **DeepFall AI**, a real-time fall detection and prevention system designed for **elderly care, healthcare, and industrial safety**. The project has been **published in IJRASET (Vol. 13, Issue IV, April 2025)** under the title:  
**"AI-Driven Heuristic Algorithms for IoT-Based Real-Time Fall Detection and Prevention."**

📄 [Read the Paper](https://www.ijraset.com/ijraset-volume/volume13-issueiv-april2025)  

---

## 🚀 Project Overview  
Falls are a leading cause of injury among the elderly and individuals with mobility impairments. Existing fall detection methods (vision-based, deep learning, or manual systems) suffer from issues like **privacy concerns, high computational costs, and delayed response times**.  

**DeepFall AI** addresses these challenges by combining:  
- **Edge Computing on ESP32-S3** (low power, real-time).  
- **MPU6050 IMU + KY-039 Heart Rate Sensor** for motion + physiological monitoring.  
- **Heuristic Decision Tree Algorithm** derived from machine learning for lightweight, rule-based inference.  
- **Multi-Channel Alerts** via buzzer, NeoPixel LEDs, and **IoT-based Telegram notifications (IFTTT integration)**.  

This approach ensures **fast detection, privacy preservation, low energy consumption, and high reliability** in real-world deployments.  

---

## 🛠️ Tech Stack  
- **Hardware:** ESP32-S3 DevkitC1, MPU6050 (IMU), KY-039 (Heart Rate), Buzzer, NeoPixel  
- **Software:** MicroPython, Thonny IDE  
- **AI/ML:** Rule-Based Decision Tree (optimized from ML models), Random Forest (benchmark)  
- **IoT Integration:** IFTTT Webhooks → Telegram Alerts  
- **Dataset:** HIHD (Hybrid Inertial & Heart Data) + custom test data  

---

## 📊 Key Features  
- ✅ Real-time fall detection using motion + heart rate features.  
- ✅ Lightweight **rule-based decision tree** optimized for ESP32-S3.  
- ✅ Accuracy: **91.43% (Decision Tree)**, **94.29% (Random Forest benchmark)**.  
- ✅ Immediate **buzzer + LED alerts** for local notification.  
- ✅ **Instant Telegram alerts** via IoT (IFTTT Webhooks).  
- ✅ Low power consumption (<100mW in active sensing).  
- ✅ Privacy-preserving (no cameras, no cloud dependency).  

---

## ⚙️ Setup & Usage  

### 1. Hardware Setup  
- ESP32-S3 DevkitC1  
- MPU6050 (connected via I²C: SCL=GPIO13, SDA=GPIO14)  
- KY-039 Heartbeat sensor (analog input pin)  
- Buzzer + NeoPixel ring for alerts  

### 2. Firmware Deployment  
- Install **MicroPython** on ESP32-S3.  
- Use **Thonny IDE** to upload `firmware/main.py`.  
- Configure Wi-Fi and IFTTT webhook in the script.  

### 3. IoT Alert Setup  
- Create an **IFTTT applet** → Webhook → Telegram.  
- Add your unique webhook URL in the ESP32 code.  

### 4. Run & Test  
- Power on device → Calibrate sensors.  
- Perform normal activities (walking, sitting) and test simulated falls.  
- Observe buzzer, LED alerts, and Telegram notifications.  

---

## 📈 Experimental Results  
- **Decision Tree (on ESP32-S3):**  
  - Accuracy: 91.43%  
  - Recall (falls): 93% → ensures minimal missed falls  
  - F1-Score: 82%  

- **Random Forest (benchmark):**  
  - Accuracy: 94.29%  
  - Higher precision but unsuitable for microcontroller deployment.  

---

## 🔮 Future Enhancements  
- TensorFlow Lite for microcontrollers (lightweight deep learning).  
- Multi-sensor fusion (barometer, GPS, thermal sensors).  
- Personalized thresholds via user feedback (adaptive learning).  
- Integration with healthcare dashboards and mobile apps.  
- Expansion to detect seizures, fainting, or long inactivity.  

---

## 📜 Publication  
This work has been published in:  
**International Journal for Research in Applied Science and Engineering Technology (IJRASET), Vol. 13, Issue IV, April 2025**  

📄 [Read Paper](https://www.ijraset.com/ijraset-volume/volume13-issueiv-april2025)  

---
