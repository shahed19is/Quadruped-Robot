# Interruptible Quadruped Robot Firmware  
### Real-Time Bluetooth Controlled Quadruped Robot with Interruptible Gait (PCA9685 Based)

**Author:** Shahed Islam  
---

## 📌 Project Overview

This project implements a real-time, Bluetooth-controlled quadruped robot firmware featuring an **interruptible gait system**.  
Unlike traditional walking robots that complete a full movement cycle before reacting to new commands, this firmware continuously monitors incoming commands and immediately interrupts any ongoing motion when a new instruction is received.

This makes the robot:

- Highly responsive  
- Safer to operate  
- More natural in motion  
- Suitable for research-grade experimentation  

The firmware is developed in an evolutionary manner, starting from a stable base version and extending toward advanced physical interaction with the environment.

---

## 🧬 Firmware Versioning and Evolution

This project follows a structured firmware evolution model:

1. **v1.0.0 – Base Firmware (No Magnetic System)**  
   Establishes the complete interruptible gait system, Bluetooth control, servo safety, and professional firmware structure.

2. **v1.1.0 – Magnetic Extension Firmware**  
   Enhances the base system by integrating an electromagnetic foot adhesion mechanism for improved stability and surface grip.

This approach ensures:
- Clear development milestones  
- Feature traceability  
- Research reproducibility  
- Professional version management  

---

## 🔵 Version 1.0.0 – Base Firmware (No Magnetic System)

**Firmware Version:** `v1.0.0`  
This is the baseline and stable release of the quadruped robot firmware.  
It implements the complete walking, control, and interruptible gait mechanism without any magnetic foot system.

The base firmware controls a 4-legged robot using 12 servo motors through the PCA9685 servo driver and communicates wirelessly using Bluetooth (HC-05).

### 🎯 Core Features (No Magnetic System)

- **Interruptible Motion Control**  
  Every movement phase checks for new commands and instantly switches behavior.

- **Real-Time Bluetooth Command Processing**  
  The robot only keeps the newest command and clears all older buffered data.

- **PCA9685 Servo Driver Integration**  
  Controls up to 16 channels with stable PWM timing.

- **Modular Configuration Design**  
  Hardware calibration and logic are separated using:
  - `robot_config.h`
  - `robot_config.cpp`

- **Servo Safety Validation**  
  Firmware halts if unsafe PWM values are detected.

- **Professional Firmware Structure**  
  Follows embedded-system best practices.

---

## 🧠 Control Commands

| Command | Action |
|------|------|
| F | Walk Forward |
| B | Walk Backward |
| L | Turn Left |
| R | Turn Right |
| V | Stand |
| S | Stop and Return to Stand |

Commands are sent via:
- HC-05 Bluetooth Module  
- Arduino Serial Monitor  

---

## 🏗 Hardware (Base Version – No Magnet)

- Arduino (Uno / Mega / Nano compatible)
- PCA9685 16-Channel Servo Driver
- 12 × Servo Motors
- HC-05 Bluetooth Module
- 4-Leg Quadruped Mechanical Frame
- External power supply for servos

---

## 📂 Firmware Structure (Base Version)

firmware/
├── firmware.ino
├── robot_config.h
└── robot_config.cpp

| File | Purpose |
|------|--------|
| `.ino` | Motion logic and command handling |
| `robot_config.h` | Hardware declarations |
| `robot_config.cpp` | Calibration and safety parameters |

---

## ⚠️ Safety Design

Before startup, the firmware checks:

SERVO_MIN ≤ PWM ≤ SERVO_MAX

If any invalid value is detected:

- The firmware halts immediately  
- A warning message is printed  
- Hardware damage is prevented  

This protects both the servos and the mechanical structure.

---

## 🔄 How the Interruptible Gait Works

1. Receive command from Bluetooth  
2. Clear old commands from buffer  
3. Keep only the newest command  
4. Interrupt current motion if command changes  
5. Execute the new motion immediately  
6. Repeat continuously  

This design gives the robot professional-grade responsiveness.

---

## ▶️ Getting Started (Base Version)

1. Open `firmware.ino` in Arduino IDE  
2. Install required library:
   - Adafruit PWM Servo Driver  
3. Connect hardware  
4. Upload firmware  
5. Send commands using Bluetooth or Serial Monitor  

---

## 🟢 Version 1.1.0 – Electromagnetic Foot Adhesion System

Version **v1.1.0** extends the base firmware by adding an **electromagnetic foot adhesion system**.  
Each leg is equipped with an electromagnet that:

- Releases before lifting the leg  
- Activates after placing the foot  
- Increases stability and grip  
- Enables operation on metallic surfaces  

This system is ideal for research involving:

- Surface adhesion  
- Stability optimization  
- Advanced locomotion control  

---

## 🔩 Additional Hardware for Magnetic Version

- 4 electromagnets  
- Relay module or MOSFET drivers  
- Separate power supply for magnets  

---

## 📂 Magnetic Firmware Structure

firmware/
├── firmware.ino
├── robot_config.h
└── robot_config.cpp

Additional features include:

- Individual magnet control per leg  
- Safe magnet ON/OFF sequencing  
- Integrated with interruptible gait logic  
- Automatic magnetic engagement on stance  

---

## 🌟 Why This Project Is Special

This project combines two advanced robotics concepts:

| Feature | Importance |
|--------|-----------|
| Interruptible gait | Real-time professional robot control |
| Electromagnetic adhesion | Experimental research capability |

Together they create a powerful experimental platform for advanced robotics.

---

## 📜 License

MIT License  
Free to use, modify, and distribute with attribution.

---

## 📝 Final Note

This is not just an Arduino project.  
It is a structured, safety-aware, real-time robotics firmware system designed using professional embedded-system principles.
