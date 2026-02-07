# HazMapper 🤖⚠️



**HazMapper** is an autonomous mobile robot developed to **map and monitor hazardous environments**, reducing human exposure to dangerous conditions.  

This project was developed as part of **Projeto Integrador II** in the **Bachelor’s Degree in Industrial Electronics and Computers Engineering** at the **University of Minho**.



---



## 🚀 Overview



HazMapper is designed to operate in risky or hard-to-access areas, collecting **environmental data** and transmitting it remotely in real time.  

The robot combines **omnidirectional locomotion**, **sensor fusion**, **wireless communication**, and a **real-time operating system** to ensure safe and responsive operation.



---



## 🔧 Main Features



- 🛞 **Omnidirectional locomotion** with 4 independently controlled motors  

- 🎯 **Closed-loop motor control** using individual PID controllers with encoder feedback  

- 🌡️ **Environmental monitoring**

&nbsp; - Temperature (HDC1080)

&nbsp; - Gas levels: CO₂ (eCO₂) & TVOC (CCS811)

- 🚧 **Obstacle detection** using ultrasonic parking sensors  

- 📷 **Onboard camera** (OV7670) for remote visual monitoring  

- 🔋 **Battery voltage monitoring** with ADC and safety indicators  



---



## 📡 Communication & Control



- 📱 **Android application (Kotlin)**

&nbsp; - Virtual joystick for omnidirectional control

&nbsp; - ON/OFF safety control

- 🔵 **Bluetooth Low Energy (BLE)** for low-latency control

- 📶 **Wi-Fi (ESP32)** for sensor data transmission via UDP

- 🧠 **STM32H755** as main controller with **FreeRTOS**



---



## 🧩 System Architecture



- **STM32H755**  

&nbsp; - Motor control, sensors, RTOS, camera interface  

- **ESP32**

&nbsp; - BLE communication with Android app  

&nbsp; - Wi-Fi data transmission

- **Custom 4-layer PCB**

&nbsp; - Power & signal isolation

&nbsp; - EMI reduction

&nbsp; - Integrated motor drivers and sensors



---



## 🛠️ Technologies Used



- C / Embedded C

- FreeRTOS

- STM32 HAL

- ESP32 (BLE & Wi-Fi)

- Kotlin (Android)

- PID control

- DMA, ADC, UART, I2C

- Custom PCB design



---



## 🔮 Future Work



- ROS2 / micro-ROS integration (digital twin)

- MJPEG video streaming over Wi-Fi

- GPS integration

- Long-range communication (LoRa)

- More powerful processing platform (e.g. Raspberry Pi)



---



## 👥 Authors



- André Martins  

- Álvaro Silva  

- Ana Cruz  

- Mariana Martins  



**Supervisor:** Prof. Adriano Tavares



---



## 📍 Institution and Degree



University of Minho  

School of Engineering



Bachelor's in Industrial Electronics and Engineering

Developed under the Integrated Project 2 subject

