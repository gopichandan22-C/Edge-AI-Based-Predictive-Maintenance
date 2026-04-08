# 🚀 Edge AI Predictive Maintenance System

## 📌 Overview
This project implements an Edge AI-based predictive maintenance system using an STM32 microcontroller. It monitors machine health in real time and detects anomalies without cloud dependency.

---

## 🧩 Components
- STM32 Microcontroller  
- MPU6050 (Vibration Sensor)  
- DHT11 (Temperature & Humidity)  
- BMP180 (Pressure Sensor)  
- LEDs (Status Indication)  
- UART (Communication)  

---

## ⚙️ Working
1. Sensors collect vibration and environmental data  
2. STM32 processes the data  
3. NanoEdge AI learns normal behavior  
4. Detects anomalies in real time  
5. Output:
   - Normal → No alert  
   - Abnormal → LED ON + UART transmission  

---

## ✨ Features
- Real-time anomaly detection  
- Runs completely on device (Edge AI)  
- No cloud required  
- Low power consumption  
- Fast response  

---

## 🧪 Testing
- Tested using a small fan to simulate machine behavior  
- Successfully detects abnormal conditions  

---

## 📊 Applications
- Industrial equipment monitoring  
- Predictive maintenance  
- IoT and embedded systems  
