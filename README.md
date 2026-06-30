<div align="center">

# 🛰️ Universal Avionics Core

### Modular. Hardware-Agnostic. Flight-Ready.

High-performance middleware bridging spatial sensors and autonomous flight controllers — built for the **SURGE 2026** research program at **IIT Kanpur**.

[![Python](https://img.shields.io/badge/Python-3.9%2B-blue?logo=python&logoColor=white)](https://www.python.org/)
[![Protocol](https://img.shields.io/badge/Protocol-MAVLink-orange)](https://mavlink.io/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

</div>

---

## 📖 Overview

**Universal Avionics Core** is a middleware framework designed to bridge heterogeneous spatial sensors with autonomous flight controllers through a hardware-agnostic driver interface. Developed as part of the SURGE 2026 research program at IIT Kanpur, the system targets real-time, mathematically rigorous sensor-to-aero-frame alignment suitable for autonomous aerial platforms.

## ✨ Key Features

- 🔌 **Hardware-Agnostic** — works with any sensor that implements a simple driver interface.
- ⚡ **Real-Time** — sub-millisecond latency with a fail-safe watchdog.
- 🧭 **Unified Coordinate System** — aligns sensor data into the MAVLink aero-frame via real-time extrinsic transformations and Perspective-n-Point (PnP) estimation.
- 💾 **Zero-Config Reboot** — persistent settings via JSON, no manual re-initialization required.
- 🛑 **Failsafe Telemetry** — automatically halts telemetry if jitter exceeds 400 ms.

## 🏗️ Architecture

The core runs a real-time loop that polls all registered sensor drivers and aligns their spatial output into the **MAVLink aero-frame** using extrinsic transformations computed via **Perspective-n-Point (PnP)** estimation. Camera-derived pose is recovered through `solvePnP`, with the resulting rotation matrix decomposed into orientation components via `RQDecomp3x3`, yielding a mathematically precise extrinsic alignment between sensor frames and the aircraft body frame.

## 🧪 Supported Sensors (tested)

| Sensor Type | Status | Driver |
|---|---|---|
| IMU | ✅ Tested | [`sensors/`](sensors/) |
| Camera (2D) | ✅ Tested | [`sensors/`](sensors/) |

## 🛠️ Tech Stack

| Layer | Technology |
|---|---|
| Core Language | Python 3.9+ |
| Communication Protocol | MAVLink |
| Sensor Alignment | IMU + 2D Camera (PnP / extrinsic transforms) |
| Configuration | JSON |
| Numerical Computing | NumPy |

## 🚀 Quick Start

### Prerequisites

- Python 3.9+
- pip

### Clone & Install

```bash
git clone https://github.com/codewithFaiz11/Universal-Avionics-Core.git
cd Universal-Avionics-Core
pip install -r requirements.txt
```

### Configuration

```bash
cp config/default.json config/local.json
```

### Calibration (Camera)

```bash
python calibration/ultimate_calibration.py
```

### Deploy

```bash
python core/flight_core.py
```

## 🧩 Extending the System

To add a new sensor, implement the `SensorDriver` interface:

```python
from sensors.sensor_interface import SensorDriver
import numpy as np

class MySensorDriver(SensorDriver):
    def get_vector(self):
        """
        Returns:
            position (np.ndarray): [x, y, z]
            attitude (np.ndarray): [roll, pitch, yaw]
        """
        return np.array([x, y, z]), np.array([roll, pitch, yaw])
```

Register your driver in `config/local.json` to enable it at runtime.

## 📁 Project Structure

| Directory / File | Description |
|---|---|
| `core/` | Main application logic |
| `sensors/` | Sensor driver implementations |
| `config/` | Configuration files |
| `calibration/` | Calibration utility scripts |
| `requirements.txt` | Python dependencies |
| `README.md` | This file |

## 🤝 Contributing

Contributions are welcome! Please follow the standard fork → branch → pull request workflow, and ensure new sensor drivers include basic tests where possible.

## 📄 License

Distributed under the MIT License. See [`LICENSE`](LICENSE) for details.

## 🙏 Contact & Acknowledgements

- **Project Lead:** Mohd Faiz
- **Supervisor:** Prof. Ketan Rajawat
- **Affiliation:** IIT Kanpur
- **Special Thanks:** SURGE 2026 initiative
