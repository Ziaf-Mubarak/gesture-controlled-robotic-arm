# Gesture-controlled-robotic-arm
# Gesture Controlled Robotic Arm using Dual ESP32 and ESP-NOW

This project implements a *wireless gesture-controlled robotic arm* using:

- An *ESP32-based glove* with an MPU6050 IMU and a flex sensor
- An *ESP32-based robotic arm controller* driving 5 servo motors
- *ESP-NOW* for low-latency communication between glove and arm

Hand orientation (pitch & roll) and a finger bend (flex) are mapped to the arm's
base, shoulder, elbow, wrist, and gripper motion. The arm moves in smooth steps
instead of jumping directly to raw values, giving more stable and natural motion.

---

## ✨ Features

- Dual ESP32 nodes communicating over *ESP-NOW*
- *MPU6050* for gesture sensing (pitch and roll)
- *Flex sensor* for gripper control (open / close)
- *5-DOF robotic arm*:
  - Base
  - Shoulder
  - Elbow
  - Wrist
  - Gripper
- Step-wise servo motion for smooth, jitter-free behaviour

---

## 🧰 Hardware Used

### Glove (Transmitter)

- ESP32 development board
- MPU6050 6-axis accelerometer + gyroscope
- Flex sensor (bending finger to control gripper)
- Resistors for flex sensor voltage divider
- Power source (e.g. 5 V USB or battery pack)
- Connection wires and glove

### Robotic Arm (Receiver)

- ESP32 development board
- 3x MG996R (or similar high-torque servos) for base, shoulder, elbow
- 2x SG90 (or similar micro servos) for wrist and gripper
- External 5 V supply for servos (with common ground to ESP32)
- Robotic arm mechanical structure (3D printed or kit)
- Breadboard / PCB and wiring

---

## 🔌 Wiring Overview

### Glove Side (Transmitter ESP32)

- MPU6050  
  - VCC → 3.3 V  
  - GND → GND  
  - SCL → GPIO 22 (default ESP32 SCL)  
  - SDA → GPIO 21 (default ESP32 SDA)

- Flex Sensor (voltage divider)  
  - One end → 3.3 V  
  - Other end → series resistor → GND  
  - Junction of flex + resistor → GPIO 34 (ADC input)

### Arm Side (Receiver ESP32)

Servos (with *external 5 V supply*):

- BASE_SERVO_PIN      → GPIO 13
- SHOULDER_SERVO_PIN  → GPIO 12
- ELBOW_SERVO_PIN     → GPIO 14
- WRIST_SERVO_PIN     → GPIO 27
- GRIPPER_SERVO_PIN   → GPIO 26

> ⚠ Important:  
> - Servo *V+* → 5 V external supply  
> - Servo *GND* → external supply GND and ESP32 GND *must be common*

---

## 🧩 Software Setup

You can use the *Arduino IDE* with ESP32 support.

1. Install *ESP32 board package* in Arduino IDE
2. Install required libraries:
   - ESP32Servo
   - Adafruit_MPU6050
   - Adafruit Unified Sensor
3. Clone this repository:
   ```bash
   git clone https://github.com/<your-username>/gesture-controlled-robotic-arm.git
