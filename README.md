# Vision-Based Autonomous Laser Tracking & Pan-Tilt Targeting System

An integrated computer vision and embedded robotics platform designed for autonomous object segmentation, real-time spatial coordinate mapping, and 3-DOF analytical inverse kinematics targeting via an actuated pan-tilt laser turret.

Author: Sila Ekincioglu  
Project: Intelligent Robotics Project (Computer Engineering)

[![C#](https://img.shields.io/badge/C%23-239120?style=for-the-badge&logo=c-sharp&logoColor=white)](https://docs.microsoft.com/en-us/dotnet/csharp/)
[![OpenCV](https://img.shields.io/badge/OPENCV_EMGU_CV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](https://www.emgu.com/)
[![Arduino](https://img.shields.io/badge/ARDUINO-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Embedded](https://img.shields.io/badge/EMBEDDED_SYSTEMS-333333?style=for-the-badge)](https://github.com/silaekincioglu/Autonomous-Laser-Turret)
[![UART](https://img.shields.io/badge/UART_PROTOCOL-238636?style=for-the-badge)](https://github.com/silaekincioglu/Autonomous-Laser-Turret)

---

## 1. Executive Summary & Problem Statement

Target acquisition and autonomous physical tracking present substantial engineering challenges in bridging real-time 2D pixel spaces with 3D physical coordinate planes. Conventional open-loop systems suffer from latency, spatial distortion, and hardware misalignment.

This project delivers a closed-loop, deterministic electro-optical targeting system combining desktop computer vision (C# / Emgu CV) with an embedded physical actuator (Arduino Uno). The software identifies high-contrast objects, isolates their centroid through spatial moments, maps pixel coordinates into physical metrics, calculates servo joint angles via analytical inverse kinematics, and transmits a 3-byte command packet across serial UART to drive a 2-axis physical laser turret.

---

## 2. Key Engineering Highlights

### Computer Vision Pipeline & Custom Morphology
- Real-time video frame acquisition via Emgu CV (C# .NET wrapper for OpenCV).
- Color-space conversion to grayscale followed by dynamic inverse binary thresholding (`ThresholdBinaryInv`).
- Custom pixel-level $3\times3$ neighborhood erosion (`Corrosion`) routine engineered to filter noise artifacts without external framework overhead.
- Centroid extraction using zero-th and first-order spatial moments ($m_{00}, m_{10}, m_{01}$) yielding the exact Center of Gravity (COG):
  $$\text{COG}_X = \frac{m_{10}}{m_{00}}, \quad \text{COG}_Y = \frac{m_{01}}{m_{00}}$$

### Metric Spatial Mapping & Analytical Inverse Kinematics
- Converts image space coordinates to real-world metric values ($X_{\text{cm}}, Y_{\text{cm}}$) using an optical offset center and scaled distance calibration:

  $$\text{Scale} = \frac{\text{Distance}}{365}$$

- Models the physical turret using link-length constants ($l_1 = 3\text{ cm}, d_1 = 4\text{ cm}$) and computes pan ($\theta_1$) and tilt ($\theta_2$) servo angles using two-stage analytical atan2 formulations:

  $$\theta_1 = \text{atan2}(P_y, P_x)$$

  $$P_{x'} = P_x - l_1 \cdot \cos(\theta_1), \quad P_{z'} = P_z + d_1$$

  $$\theta_2 = \pi - \text{atan2}(P_{x'}, P_{z'})$$

- Enforces strict hardware angle clamping ($[0^\circ, 180^\circ]$) to safeguard servo gearboxes against out-of-bounds mechanical strain.

### Serial Protocol & Controlled Firing Sequence
- Transmits a compact 3-byte binary payload over UART at 9600 baud:
  `Buff[0] = Th1 (Pan Angle), Buff[1] = Th2 (Tilt Angle), Buff[2] = LaserState`
- Arduino firmware reads validated angle bytes, sweeps the servos, flushes serial buffer overflows, and energizes the laser diode.
- Windows Forms layer utilizes a 3-stage asynchronous timer architecture (`timer1`, `timer2`, `timer3`) to provide a 2-second pulse duration, protecting the laser diode from thermal degradation.

---

## 3. System Architecture


```

[ Overhead Camera Feed ]
│
▼
[ Image Preprocessing: Grayscale & Inverse Binary Thresholding ]
│
▼
[ Custom Morphological Filter & Centroid Extraction (Moments) ]
│
▼
[ Pixel-to-Centimeter Coordinate Transform (Xcm, Ycm, Px, Py, Pz) ]
│
▼
[ 2-DOF Analytical Inverse Kinematics Engine (Theta1, Theta2) ]
│
▼
[ 3-Byte UART Serial Packet: Th1, Th2, Laser Activation ]
│
▼
[ Arduino Uno Microcontroller ] ───> [ 2x Servos (Pan/Tilt) + Laser Diode ]

```

---

## 4. Hardware Configuration

- Microcontroller: Arduino Uno (ATmega328P)
- Actuators: 2x Micro Servos (Pan Axis on Pin 2, Tilt Axis on Pin 5)
- Emitter: 5V Red Focusable Laser Diode Module (Digital Pin 13)
- Bridge: CH340/CH341 USB-to-UART Interface (9600 Baud)

---

## 5. Repository Structure

To preserve proprietary implementation details, this repository serves as a focused engineering showcase:


```

├── Form1.cs                    # Image processing pipeline, moments, and inverse kinematics
├── black_shooting_arduino.ino  # Microcontroller servo control and laser pulse trigger
└── README.md                   # System documentation and architectural specifications

```

---
