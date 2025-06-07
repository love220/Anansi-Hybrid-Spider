# Anansi-Hybrid-Spider
This robot is inspired from West African folklore. Anansi is a spider trickster god known for intelligence and storytelling. 
Here's a GitHub-ready description (README-style) for your quadruped walking robot project using inverse kinematics:

---
This project implements a simple quadruped robot with **4 legs** (each having a hip and a knee joint), capable of **forward walking using inverse kinematics (IK)**. The robot uses SG90 micro servos driven via the **PCA9685 servo driver**, controlled through an Arduino-UNO board.

### ✨ Features

* 🔧 **Auto-calibration** of all hips and knees to 90° at startup.
* 🤖 **Smooth forward walking** motion using trigonometric inverse kinematics.
* 🦿 Each leg is a 2-DOF manipulator with upper and lower leg segments.
* 🎮 Easily extendable to add crawl, turn, dance, or wave gestures.
* 📦 Compact and clean code architecture with clear leg indexing.

---

### 🧠 Hardware Used

* **Arduino UNO**
* **PCA9685 16-channel PWM Servo Driver**
* **8x SG90 Micro Servos**
* **12V to 5V Buck Converter (if needed for servo power)**
* **Optional: 7.4V 2S LiPo or 5V USB power bank**

---

### ⚙️ Leg Configuration

Each leg is composed of:

* **Hip Joint** – Moves leg forward/backward (X-axis).
* **Knee Joint** – Moves leg up/down (Y-axis).

Legs are indexed as:

```
0: Front Left (FL)
1: Front Right (FR)
2: Back Left (BL)
3: Back Right (BR)
```

---

### 📐 Inverse Kinematics

The code solves the position `(x, y)` of the foot to determine:

* `theta1`: Hip angle
* `theta2`: Knee angle

This makes the motion smoother and more lifelike, with better control of stride and lift height.

---

### 🔌 How to Use

1. Upload the code via Arduino IDE.
2. Open Serial Monitor for debug logs.
3. On boot, the robot calibrates all servos to 90°.
4. It starts crawling forward in a loop.

You can modify the `loop()` function to introduce commands like `crawl`, `dance`, or even `turn`.

---

### 🛠️ Future Enhancements

* Add Bluetooth/Wi-Fi control
* Add obstacle detection (e.g., ultrasonic sensor)
* Implement gait switching (tripod, ripple, etc.)
* Integrate battery monitoring and safety shutdown
* Add web/GUI control via ESP8266

---
