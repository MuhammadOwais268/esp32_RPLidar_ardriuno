# 🛰️ 2D LiDAR UDP Data Receiver and Real-Time Visualizer

**Summary**: This Python tool receives binary UDP packets from an ESP32-based 2D LiDAR sensor, decodes them, and visualizes the data in real-time. It features a threaded UDP receiver, checksum validation, and four live plots: polar and Cartesian views, plus distance and quality histograms, making it ideal for robotics and LiDAR debugging.

---

## 📘 Overview

This program enables **real-time LiDAR scanning visualization** directly on your computer using UDP communication. It is ideal for:
- ESP32-based 2D LiDAR sensors transmitting scan data over Wi-Fi.
- Robotics projects involving mapping, obstacle detection, or SLAM.
- Data debugging and calibration of LiDAR distance sensors.

---

## ⚙️ Features

- ✅ **UDP Socket Receiver**: Listens to LiDAR binary packets over a specified UDP port.
- ✅ **Binary Packet Decoder**: Decodes angle, distance, and quality values with checksum validation.
- ✅ **Real-Time Visualization**:
  - Polar (radial) scan view
  - Cartesian (top-down) 2D map
  - Distance and quality histograms
  - Live stats display (points, packets, errors, averages)
- ✅ **Threaded Receiver**: Runs a background thread for non-blocking data reception.
- ✅ **Data Filtering**: Ignores invalid or stale points and limits visualization to a configurable range.
- ✅ **Dynamic Histograms & Quality Color Mapping**: Plots real-time quality levels (0–255) using a `jet` colormap.

---

## 🧩 System Architecture
┌─────────────────────────────┐
│          ESP32 LiDAR        │
│ ↳ Sends UDP binary packets  │
│   (Angle, Distance, Q)      │
└──────────────┬──────────────┘
│ UDP
▼
┌─────────────────────────────┐
│    Python Receiver Thread   │
│ - Socket bind to port 4210  │
│ - Decode & validate data    │
│ - Store in shared buffer    │
└──────────────┬──────────────┘
│ Shared Memory
▼
┌─────────────────────────────┐
│     Real-Time Visualizer    │
│ - Polar view                │
│ - Cartesian 2D map          │
│ - Histograms                │
│ - Stats display             │
└─────────────────────────────┘

---

## 📡 LiDAR UDP Packet Structure

Each UDP packet follows this **binary format**:

| Field         | Size (bytes) | Description                              |
|---------------|--------------|------------------------------------------|
| Header        | 2            | Magic bytes `0xAA 0x55`                 |
| Point Count   | 1            | Number of LiDAR points in this packet    |
| Point Data    | 5 × N        | Each point = `angle_x100` (2B) + `distance_mm` (2B) + `quality` (1B) |
| Checksum      | 1            | XOR of all previous bytes               |

### Example
[AA][55][03][90 01][E8 03][64][20 03][58 02][50][B3]

This represents:
- 3 data points
- Each containing (angle × 100, distance in mm, quality)
- Ending with checksum byte `0xB3`

---

## 🖥️ Visualization Layout

| Plot Type             | Description                                              |
|-----------------------|----------------------------------------------------------|
| **Polar Plot**        | Displays points in a circular (radial) format with color representing signal quality. |
| **Cartesian Plot**    | Converts polar coordinates to X-Y for top-down LiDAR view. |
| **Distance Histogram**| Shows frequency distribution of measured distances.       |
| **Quality Histogram** | Displays quality level distribution across all readings.  |

### Example Layout
Polar View           Cartesian View
Distance Histogram   Quality Histogram

---

## 🧠 How It Works

1. **ESP32 LiDAR** sends UDP packets containing LiDAR scan points.
2. **`LidarReceiver`** binds to the UDP port, listens for incoming packets, and decodes them.
3. Each valid point (angle, distance, quality) is timestamped and stored.
4. **`LidarVisualizer`** retrieves recent points (last 1 second) and updates:
   - Polar scatter plot
   - Cartesian scatter plot
   - Distance and quality histograms
5. The display refreshes every 50 ms using Matplotlib’s `FuncAnimation`.

---
### 1. ### 1. Install Dependencies
```bash
pip install numpy matplotlib
```

### 2. Run Program
```bash
python3 hello.py
```
### 3. Configure Your ESP32
Ensure your ESP32 sends UDP packets to your computer’s IP address on port 4210 in the defined binary format.
### 4. Visualize
Once data starts streaming, the GUI will show:

Live scanning points
Distance and quality histograms
Real-time stats
Press Ctrl + C in the terminal to exit.

### 5.⚙️ Configuration Parameters

Parameter,Description,Default
UDP_PORT,Port to receive LiDAR packets,4210
BUFFER_SIZE,Max UDP buffer size (bytes),1024
MAX_DISTANCE,Maximum range to visualize (mm),8000
HISTORY_SIZE,Number of past points to keep,360

### 6.Code Structure

lidar_visualizer.py
├── LidarReceiver
│   ├── decode_packet()   → Parse UDP binary packets
│   ├── receive_loop()    → Background thread
│   ├── get_points()      → Return active LiDAR points
│   ├── start() / stop()  → Thread control
│
├── LidarVisualizer
│   ├── update()          → Refresh plots
│   ├── show()            → Start live animation
│
└── main()                → Program entry point

### 7. 🧪 Example Console Output
============================================================
2D LiDAR UDP Receiver and Visualizer
============================================================
Listening on UDP port 4210
Maximum display distance: 8000mm (8.0m)

Waiting for LiDAR data...
Press Ctrl+C to exit

LiDAR Receiver initialized on port 4210
Starting receiver thread...
Packets received: 100, Errors: 0, Active points: 360
Packets received: 200, Errors: 1, Active points: 720

## 🧰 Troubleshooting
Issue,Possible Cause,Solution
No points visible,ESP32 not sending data / wrong IP,Check UDP port and IP configuration
Checksum errors,Incorrect packet formatting,Verify your checksum XOR implementation
Laggy visualization,Low framerate or large data burst,Increase update interval or reduce HISTORY_SIZE
“Socket timeout” messages,No data received in 1 second,Normal; ignored automatically

## 🧑‍💻 Author
Muhammad Owais
Electrical Engineering, UET Taxila
Project: Real-Time ESP32 LiDAR Data Visualization System

## 🌟 Contribute
If you find this project useful, please:

⭐ Star the repository
🐛 Report bugs or suggest features
🧠 Fork and enhance it for your own LiDAR setup
