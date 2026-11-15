# Team Motorex - Vehicle Telemetry Dashboard (SAE Baja 2026)

Core initialization and vehicle telemetry OS graphics controller for SAE Baja 2026, built with **LVGL 9.x** and **SDL2** with live **SocketCAN** integration for 1024×600 displays.

---

## Demo Video

https://github.com/user-attachments/assets/
> **Video Demonstration**: The video file is included in this repository.

<p align="center">
  <video src="Motorex_dashbaord_video.mp4" width="85%" controls autoplay loop muted>
    Your browser does not support the video tag.
  </video>
</p>

[▶ Click here to view or download the demo video (`Motorex_dashbaord_video.mp4`)](Motorex_dashbaord_video.mp4)

---

## Key Features

- **Vehicle Telemetry Display**: Real-time vehicle speed, RPM, battery voltage, motor & controller temperatures, throttle and regenerative braking percentages.
- **Steer-By-Wire (SBW) Control**: Live steering angle visualization, encoder calibration offset adjustment, and integrated on-screen target angle numeric keypad (-60° to +60°).
- **Autonomous & Driving Modes**: Selection interface for Standby, AEB (Autonomous Emergency Braking), LKA (Lane Keep Assist), and Endurance modes with active CAN broadcast to Jetson Nano.
- **Sensor & Error Diagnostics**: Live sensor health monitoring, AS5600 magnetic encoder status, CAN bus connectivity checks, and critical fault alerts.
- **Native 1024×600 Layout**: Tailored for 7-inch automotive touchscreens and PC simulation.

---

## CAN Bus Integration

| Subsystem | CAN ID | Direction | Description |
| :--- | :--- | :--- | :--- |
| **Steering Command** | `0x103` | Linux → Teensy | Target steering angle command (uint16 encoded, LE) |
| **Steering Feedback** | `0x203` | Teensy → Linux | Live steering angle feedback |
| **Steering Status** | `0x063` | Teensy → Linux | AS5600 encoder status (`0` = Nominal, `1` = Error) |
| **Autonomous Mode** | `0x400` | Linux → Jetson | Driving mode selection command |
| **Motor Controller 1** | `0x180117EF` | MCU → Meter | Bus voltage, bus current, phase current, RPM |
| **Motor Controller 2** | `0x180217EF` | MCU → Meter | Temps (controller/motor), throttle %, brake %, gear |

---

## Getting Started

### Prerequisites

#### Linux (Debian / Ubuntu / WSL2)
```bash
sudo apt update
sudo apt install build-essential cmake libsdl2-dev
```

#### Verification
```bash
sdl2-config --version
cmake --version
gcc --version
```

---

## Building and Running

### Command Line (CMake)

1. Clone repository with submodules:
   ```bash
   git clone --recursive <repository-url>
   cd Motorex_vehicle_dashbaord
   ```
   *(If already cloned without `--recursive`, run `git submodule update --init --recursive`)*

2. Configure and build:
   ```bash
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   ```

3. Run the application:
   ```bash
   ../bin/main
   ```

### Visual Studio Code

1. Open `simulator.code-workspace` in VS Code (`File -> Open Workspace from File...`).
2. Press `Ctrl+Shift+B` (or run task `Build and Run`).
3. To debug with GDB, switch to the Run & Debug tab and select `Debug LVGL demo with gdb` (or press `F5`).

---

## Project Structure

```
├── Motorex_dashbaord_video.mp4    # Video demonstration of the dashboard UI
├── CMakeLists.txt                 # CMake build configuration
├── lv_conf.h                      # LVGL feature configuration
├── simulator.code-workspace       # VS Code workspace settings and launch profiles
├── src/
│   ├── main.c                     # Core telemetry application and UI layouts
│   ├── motorex_bat.c              # Battery UI graphic asset
│   ├── steering_wheel.c           # Steering wheel graphic asset
│   ├── car_chassis.c              # Vehicle chassis graphic asset
│   ├── throttle_pedal.c           # Throttle pedal graphic asset
│   ├── brake_rotor.c              # Brake rotor graphic asset
│   ├── hal/                       # Hardware Abstraction Layer (SDL2 drivers)
│   └── freertos/                  # FreeRTOS port files (optional)
├── FreeRTOS/                      # FreeRTOS submodule (optional kernel)
└── lvgl/                          # LVGL 9.x submodule
```

---

## License

This project is licensed under the MIT License - see [licence.txt](licence.txt) for details.
