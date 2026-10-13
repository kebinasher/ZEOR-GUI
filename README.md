# 🛰️ Zenith Orbitron Ground Station (ZEOR GUI)

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![GUI](https://img.shields.io/badge/Framework-PyQt6-green.svg)](https://riverbankcomputing.com/software/pyqt/)
[![Plotting](https://img.shields.io/badge/Plots-PyQtGraph-orange.svg)](https://www.pyqtgraph.org/)
[![Hardware](https://img.shields.io/badge/MCU-Teensy%204.1-red.svg)](https://www.pjrc.com/teensy/teensy41.html)
[![Radio](https://img.shields.io/badge/RF-LoRa%20E22--900T22D-blueviolet.svg)](https://www.cdebyte.com/)
[![Sensor](https://img.shields.io/badge/IMU-BNO085-yellow.svg)](https://www.ceva-ip.com/)

**Zenith Orbitron Ground Station v6.0** is a high-performance desktop telemetry and command-and-control interface designed for aerospace, rocketry, and CanSat / CubeSat missions. Built with **PyQt6**, it receives and visualizes high-rate telemetry sent from a **Teensy 4.1** flight computer over an **Ebyte LoRa E22** radio link.

---

## ✨ Features

- **⚡ Real-Time Telemetry Dashboard**: Fast, low-latency multi-channel sensor streaming powered by `pyqtgraph`.
- **🧭 Attitude & Heading Reference (AHRS)**: Visual Artificial Horizon, Gyroscope, and Compass displays driven by the onboard **BNO085 9-DOF IMU** (Roll, Pitch, Yaw).
- **🗺️ Live GPS Tracking & Maps**: Embedded interactive mapping with `folium` and `PyQt6-WebEngine` displaying coordinates, flight path, and ground track.
- **📡 LoRa E22 Radio Management**: Serial port auto-detection, signal strength monitoring (RSSI / SNR), and packet error rate tracking.
- **💾 Flight Logging & Playback**: Automatic logging of incoming packets with timestamps for post-flight analysis.
- **🖥️ Raw Console & Diagnostics**: Dedicated terminal tab for raw serial packet inspection and custom command uplink.

---

## 🛠️ Hardware Stack

| Component | Model / Specification | Purpose |
| :--- | :--- | :--- |
| **Flight Computer** | Teensy 4.1 (ARM Cortex-M7 @ 600 MHz) | Onboard data acquisition & flight logic |
| **RF Transceiver** | Ebyte LoRa E22-900T22D (868/915 MHz) | Long-range wireless telemetry link |
| **IMU / Navigation** | BNO085 9-DOF Sensor Fusion | Orientation, Quaternion, Euler angles, Accel |
| **UART Protocol** | 9600 Baud (8N1 Default / Configurable) | Ground station serial communication |

---

## 📦 Prerequisites & Installation

### 1. Clone the Repository
```bash
git clone https://github.com/srivatsa11826-hub/zeor-gui.git
cd zeor-gui

