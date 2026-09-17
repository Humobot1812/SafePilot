# SafePilot 🚁

### *Making drone piloting safe, simple, and accessible for everyone.*

> **Platform**: 🖥️ Ground Station / Companion Computer · Linux · ROS 2 Humble
> **Package name**: `safe_pilot`

---

## 💡 Why SafePilot?

Learning to fly a drone is hard — one wrong stick input and your drone is on the ground in pieces.

**SafePilot** removes that fear. It is an open-source ROS 2 package that wraps a gamepad/joystick (PlayStation, Xbox, or any generic USB controller) around a MAVLink flight controller (ArduPilot / PX4), adding a layer of intelligent safety and assistance so that **anyone — regardless of experience — can pick up a controller and fly confidently**.

| Problem | SafePilot's answer |
|---|---|
| 💥 Beginners crash often | Embedded geofence keeps the drone inside a safe boundary |
| 😰 Mode switching is confusing | Single button per action — arm, takeoff, land, RTL, hold |
| 📍 Hard to learn waypoint missions | One button uploads and runs an autonomous GPS mission |
| 🎮 Raw MAVLink is complex | Full joystick-to-drone bridge, plug in and fly |
| ⚡ Emergency situations | Dedicated emergency kill and RTL buttons always within reach |

**Core features:**
- 🕹️ **3D velocity teleoperation** — full body-frame control from analog sticks
- ✋ **Multi-core AI hand gesture control** — contact-free flight with translational strafe, responsive forward/reverse linear depth scaling, and closed-fist in-place yaw rotation
- 🛡️ **Embedded geofencing** — polygon boundary + altitude ceiling with auto-RTL on breach
- 🗺️ **Autonomous waypoint missions** — upload, start, pause, and resume with a single button
- 📡 **Live telemetry** — position, armed state, and flight mode streamed over ROS 2
- 🖥️ **One-command simulation** — Gazebo Harmonic + ArduPilot SITL launched together

---

## 📑 Table of Contents

1. [Architecture Overview](#️-architecture-overview)
2. [Package Structure](#-package-structure)
3. [Prerequisites & Installation](#️-prerequisites--installation)
4. [Build & Launch Instructions](#-build--launch-instructions)
5. [AI Hand Gesture Teleoperation (Multi-Core)](#-ai-hand-gesture-teleoperation-multi-core)
6. [Controller Layout & Keybindings](#-controller-layout--keybindings)
7. [ROS 2 Topics & MAVLink Endpoints](#-ros-2-topics--mavlink-endpoints)
8. [Geofencing & Autonomous Mission Setup](#️-geofencing--autonomous-mission-setup)
9. [Controller Calibration & Customization](#-controller-calibration--customization)
10. [Configuration Reference](#️-configuration-reference)

---

## 🏗️ Architecture Overview

```text
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                 Ground Station (Linux)                                 │
│                                                                                        │
│   ┌─────────────┐       /joy      ┌────────────────────────────────────────────────┐   │
│   │  joy_node   │ ──────────────► │          DroneJoyTeleop (ROS 2 Node)           │   │
│   │ (HID driver)│                 │                                                │   │
│   └─────────────┘                 │  • Sticks  → 3D velocity set-points            │   │
│                                   │  • Buttons → discrete flight commands          │   │
│   ┌─────────────┐   /drone_status │  • D-pad   → live speed scaling                │   │
│   │  Subscriber │ ◄────────────── │  • SELECT+B → toggle Hand Gesture Mode         │   │
│   └─────────────┘                 │  • 50 Hz Timer → thread-safe gesture drainage  │   │
│                                   └───────────────┬────────────────────────────────┘   │
│                                                   │ asyncio.run_coroutine              │
│                                   ┌───────────────▼────────────────┐                   │
│                                   │    DroneController (MAVSDK)    │                   │
│                                   │  • cmd_arm_takeoff / land / rtl│                   │
│                                   │  • Offboard loop (20 Hz)       │                   │
│                                   │  • Mission & Geofence handlers │                   │
│                                   └───────────────┬────────────────┘                   │
│                                                   │                                    │
│   ┌───────────────────────────────────────────────┴────────────────────────────────┐   │
│   │ Process 2: GestureVisionProcess (Dedicated Multi-Core Worker)                  │   │
│   │                                                                                │   │
│   │   • CPU Affinity Pinning: Dedicated CPU cores (e.g. Cores 20–23)               │   │
│   │   • Camera: Low-latency VideoCapture (CAP_PROP_BUFFERSIZE=1)                   │   │
│   │   • MediaPipe: Hand Landmarker via TFLite XNNPACK CPU Delegate                │   │
│   │   • Dual Modes: Open Hand (Strafe/Climb/Depth) vs Closed Fist (Yaw)            │   │
│   │   • Anti-Overlap HUD: Real-time 2-row telemetry + visual depth rings           │   │
│   │   • IPC: Non-blocking multiprocessing.Queue & stop Event                       │   │
│   └────────────────────────────────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────┬────────────────────────────────────┘
                                                    │ MAVLink UDP :14550
                                   ┌────────────────▼────────────────┐
                                   │ Flight Controller (ArduPilot/PX4│
                                   │ • SITL via Gazebo Harmonic      │
                                   │ • Physical drone                │
                                   └─────────────────────────────────┘
```

The package achieves high performance and rock-solid safety through strict process isolation:

- **ROS 2 spin thread** — Receives `/joy` inputs and safely runs executor callbacks.
- **50 Hz ROS 2 Timer (`_gesture_poll_callback`)** — Safely pulls velocity setpoints and status events from the vision process directly within the ROS 2 thread, eliminating DDS thread crashes.
- **MAVSDK asyncio thread** — Runs the offboard velocity loop at 20 Hz, streams drone telemetry, and executes all MAVLink commands.
- **Dedicated Vision Process (`GestureVisionProcess`)** — Pinned to dedicated CPU performance cores via `os.sched_setaffinity` to isolate AI vision inference from real-time flight communications.

---

## 📁 Package Structure

```text
SafePilot/                          # repo root / ROS 2 workspace package
├── safe_pilot/
│   ├── __init__.py
│   ├── teleop_node.py              # Classic ROS 2 gamepad teleop + MAVSDK bridge
│   ├── teleop_gesture_node.py      # Multi-Core gesture teleop + gamepad integration
│   └── hand_landmarker.task        # Bundled MediaPipe hand landmarker model
├── launch/
│   ├── simulation.launch.py        # Full pipeline: Gazebo → ArduCopter SITL → Teleop
│   ├── sim.launch.py               # Quick alias for simulation.launch.py
│   └── teleop.launch.py            # Launches joy_node + teleop_node
├── resource/
│   └── safe_pilot
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

---

## 🛠️ Prerequisites & Installation

### System Requirements

| Dependency | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 LTS | Tested on Jammy |
| ROS 2 | Humble Hawksbill | Required |
| Python | 3.10+ | Ships with Ubuntu 22.04 |
| MAVSDK-Python | Latest | `pip install mavsdk` |
| MediaPipe | 0.10+ | Required for gesture control (`pip install mediapipe`) |
| OpenCV Python | 4.x+ | Required for gesture vision (`pip install opencv-python`) |
| ArduPilot SITL | Latest | Simulation only |
| Gazebo Harmonic | 8.x | Simulation only |

### 1. Install ROS 2 Joystick Driver

```bash
sudo apt update
sudo apt install -y ros-humble-joy
```

### 2. Install Python Dependencies

```bash
pip install mavsdk mediapipe opencv-python
```

### 3. Verify Joystick Connection

Plug in your controller via USB or Bluetooth and confirm it is recognised:

```bash
ls -l /dev/input/js*
```

Optionally test raw axis/button values:

```bash
sudo apt install -y jstest-gtk
jstest /dev/input/js0
```

### 4. Simulation Dependencies *(optional)*

For the full Gazebo + ArduPilot SITL simulation, ensure these are available on your system:

- **Gazebo Harmonic** — `gz sim` command accessible
- **ArduPilot SITL** — `sim_vehicle.py` accessible (add `Tools/autotest` to `PATH`)
- **ardupilot_gazebo plugin** — built at `~/gz_ws/src/ardupilot_gazebo/build`

---

## 🚀 Build & Launch Instructions

### 1. Build the Package

From the workspace root (`~/Semi_control`):

```bash
cd ~/Semi_control
colcon build --packages-select safe_pilot
source install/setup.bash
```

---

### 2. Launch Options

#### Option A — Full Automated Simulation *(one command)*

Starts **Gazebo Harmonic**, then **ArduCopter SITL** after a configurable delay, then the **ROS 2 teleop** system:

```bash
source ~/Semi_control/install/setup.bash
ros2 launch safe_pilot simulation.launch.py
# Quick alias:
ros2 launch safe_pilot sim.launch.py
```

**Tunable parameters:**

| Parameter | Default | Description |
|---|---|---|
| `ardupilot_delay` | `3.0` s | Seconds before SITL starts after Gazebo boots |
| `teleop_delay` | `15.0` s | Seconds before teleop nodes start |
| `world` | `aerothon_ground2.sdf` | Gazebo world file (in `ardupilot_gazebo/worlds`) |

> **Note:** The default world `aerothon_ground2.sdf` is included for demonstration purposes only. Replace it with any `.sdf` world file present in your `ardupilot_gazebo/worlds` directory (e.g. `iris_runway.sdf`, or a custom world you've created).

```bash
# Example with custom delays and world file
ros2 launch safe_pilot sim.launch.py \
  ardupilot_delay:=5.0 teleop_delay:=20.0 world:=iris_runway.sdf
```

#### Option B — Teleop Only *(SITL or drone already running)*

Launches `joy_node` and `teleop_node` together:

```bash
source ~/Semi_control/install/setup.bash
ros2 launch safe_pilot teleop.launch.py
```

#### Option C — Manual / Separate Terminals

**Terminal 1 — Joystick driver:**
```bash
source ~/Semi_control/install/setup.bash
ros2 run joy joy_node
```

**Terminal 2 — Drone teleop node:**
```bash
source ~/Semi_control/install/setup.bash
ros2 run safe_pilot teleop_node
```

#### Option D — Hand Gesture Teleoperation Mode *(Multi-Core AI)*

Enables contact-free vision control alongside gamepad controls. Runs on dedicated performance CPU cores:

**Terminal 1 — Joystick driver:**
```bash
source ~/Semi_control/install/setup.bash
ros2 run joy joy_node
```

**Terminal 2 — Gesture-enabled teleop node:**
```bash
source ~/Semi_control/install/setup.bash
ros2 run safe_pilot teleop_gesture_node
```

> **Quick Toggle:** Press **SELECT (10) + B (1)** simultaneously on your controller at any time to enter or exit Hand Gesture Control mode.

> **Note:** Ensure your flight controller is broadcasting MAVLink on `udpin://0.0.0.0:14550`. Update `DRONE_ADDRESS` in [`teleop_node.py`](safe_pilot/teleop_node.py) or [`teleop_gesture_node.py`](safe_pilot/teleop_gesture_node.py) if your endpoint differs.

---

## ✋ AI Hand Gesture Teleoperation (Multi-Core)

The `teleop_gesture_node` provides contact-free piloting using your webcam and Google MediaPipe hand landmark tracking. It is engineered with a **multi-core isolated process architecture** that pins vision processing to dedicated CPU cores, completely isolating AI inference from MAVLink and ROS 2 communication loops.

### 🕹️ Entering & Exiting Gesture Mode
- **From Gamepad**: Press **SELECT (button 10) + B (button 1)** simultaneously.
- **From OpenCV GUI**: Press `'q'` inside the window or click the window manager close button (`X`).
- **Safety Restoration**: On exit, joystick control is instantly restored and the drone is placed in an automated stationary hover (`HOLD`).

---

### ⏱️ Initial Calibration
1. When activated, the OpenCV HUD enters `[ CALIBRATING... ]` mode.
2. Present your open hand flat inside the **green guide circle**.
3. Hold steady for ~3 seconds (60 frames). An animated radial yellow progress arc tracks calibration progress.
4. Once locked, the hand's neutral center ($cx_0, cy_0$) and baseline size ($s_0 = \sqrt{\text{area}_0}$) are saved, and the system transitions to `[ MODE: STRAFE ]`.

---

### ✈️ Dual Flight Modes

SafePilot supports two distinct, intuitive gesture modes based on hand shape:

#### 1. 🖐️ Open Hand Mode: 3D Translational Flight (Strafe / Climb / Forward / Reverse)
When fingers are spread open flat:

| Hand Motion | Physical Drone Response | MAVSDK Output | Speed Range | Visual Feedback |
|---|---|---|---|---|
| **Move Hand Left / Right** | Lateral Strafe Left / Right | `vy` ($\pm dx \times 5.0$) | $\pm 0.2\text{ to }3.0\text{ m/s}$ | Green velocity vector arrow |
| **Move Hand Up / Down** | Ascend / Descend | `-vz` (Up) / `+vz` (Down) | $\pm 0.2\text{ to }3.0\text{ m/s}$ | Green velocity vector arrow |
| **Push Hand Closer to Camera** | Fly Forward | `+vx` | $+0.6\text{ to }+3.0\text{ m/s}$ | Expanding **turquoise ring** + `(FWD)` HUD tag |
| **Pull Hand Away from Camera** | Fly Reverse / Backward | `-vx` | $-0.6\text{ to }-3.0\text{ m/s}$ | Contracting **orange ring** + `(REV)` HUD tag |
| **Yaw Axis** | Rotation Locked | `yaw = 0.0 deg/s` | `0.0` | Prevents unwanted rotation |

> **Linear Depth Scaling**: Forward and reverse speeds use normalized linear hand scale:
> $$\text{delta\_scale} = \frac{\sqrt{\text{area}} - \sqrt{\text{ref\_area}}}{\sqrt{\text{ref\_area}}}$$
> With `GESTURE_SENSITIVITY_FWD = 6.5 m/s`, subtle hand movements produce smooth, symmetric speeds from $0.6\text{ m/s}$ up to the $3.0\text{ m/s}$ hard safety limit, regardless of distance from the camera.

---

#### 2. ✊ Closed Fist Mode: In-Place Yaw Rotation (Pure Heading Control)
When fingers are curled into a closed fist (detected via 3D finger curl heuristics with 3-frame debounce):

| Hand Motion | Physical Drone Response | MAVSDK Output | Speed Range | Visual Feedback |
|---|---|---|---|---|
| **Move Fist Right** | Rotate Clockwise | `+yaw` | $+5.0\text{ to }+45.0^\circ\text{/s}$ | `>> YAW RIGHT >>` cyan rotation arc |
| **Move Fist Left** | Rotate Counter-Clockwise | `-yaw` | $-5.0\text{ to }-45.0^\circ\text{/s}$ | `<< YAW LEFT <<` cyan rotation arc |
| **Fist Centered** | Neutral (Heading Locked) | `yaw = 0.0` | `0.0` | `YAW NEUTRAL` indicator |
| **Strafe & Forward Axes** | Translation Locked | `vx = 0.0, vy = 0.0` | `0.0` | Eliminates unwanted drift |
| **Move Fist Up / Down** | Ascend / Descend | `vz` | $\pm 0.2\text{ to }3.0\text{ m/s}$ | Vertical velocity arrow |

---

### 🖥️ Anti-Overlap 2-Row HUD Layout

The OpenCV camera interface features a non-overlapping two-row translucent header and a dedicated footer bar designed for maximum clarity in 640×480 and HD displays:

- **Row 1 ($y = 26$)**: Left displays the active mode badge (`[ MODE: STRAFE ]` in green or `[ MODE: YAW ROTATION ]` in cyan). Right displays the hand state (`OPEN HAND` or `CLOSED FIST`), dynamically right-aligned with exact pixel margins.
- **Row 2 ($y = 54$)**: Live telemetry values:
  - Open Hand: `VX: +1.3 (FWD)   VY: -0.4   VZ: +0.0 m/s`
  - Closed Fist: `YAW: +25.5 deg/s   VZ: +0.0 m/s   (STRAFE LOCKED)`
- **Center Visual Feedback**:
  - Green guide circle ($r_g = \min(w,h) / 5$).
  - Dynamic depth ring (expanding turquoise for forward, contracting orange for reverse).
  - Rotation arcs for yaw rate.
- **Footer Bar (32px tall)**: Displays package label and hotkey instructions (`SELECT+B or 'Q' to exit`).

---

### 🛡️ Vision Safety & Fault Tolerance
- **1.5-Second Hand Loss Timeout**: If the hand leaves the frame or tracking is lost for $>1.5\text{ s}$, the system commands `DRONE HOLD` and hovers safely in place.
- **DDS Thread Safety**: Queue commands are ingested strictly through a 50 Hz ROS 2 timer (`_gesture_poll_callback`) on the executor thread, preventing ROS 2 DDS C-library segmentation faults.
- **Zero Memory Leaks**: Clean process termination bypasses slow TensorFlow Lite C++ destructor deadlocks via fast process exit and releases all resources with zero dangling shared memory segments in `/dev/shm`.
- **Headless Fallback**: If display output is unavailable (e.g., SSH session without X11 forwarding), the vision loop automatically catches GUI notices and operates headlessly without crashing.

---

## 🎮 Controller Layout & Keybindings

### 🕹️ Analog Sticks — 3D Velocity Control (Body Frame)

| Stick | Axis | Physical Motion | MAVSDK Output |
|---|---|---|---|
| **Right Stick ↑↓** | `3` | Forward / back | `+vx` Forward / `-vx` Backward |
| **Right Stick ←→** | `2` | Left / right | `-vy` Strafe Left / `+vy` Strafe Right |
| **Left Stick ↑↓** | `1` | Up / down | `-vz` Ascend / `+vz` Descend |
| **Left Stick ←→** | `0` | Left / right | `-yaw` Yaw Left / `+yaw` Yaw Right |

> Dead-band of `0.05` applied to all axes to prevent stick drift.

---

### 🎛️ D-Pad — Live Speed Tuning

| D-Pad | Effect |
|---|---|
| **↑ Up** | XY & Z speed × `1.15` (max `10.0 m/s`) |
| **↓ Down** | XY & Z speed × `0.85` (min `0.2 m/s`) |
| **← Left** | Yaw rate × `1.15` (max `60.0 deg/s`) |
| **→ Right** | Yaw rate × `0.85` (min `5.0 deg/s`) |

Default speeds at startup: **XY = 2.0 m/s · Z = 1.0 m/s · Yaw = 30.0 deg/s**

---

### 🔘 Face & Shoulder Buttons

| Button | Index | Action | Description |
|---|---|---|---|
| **Y** | `4` | `ARM + TAKEOFF` | Arms motors, climbs to `TAKEOFF_ALT` (5 m), auto-enables offboard |
| **A** | `0` | `LAND` | Initiates landing; disarms on touchdown |
| **X** | `3` | `RTL` | Exits offboard, triggers Return-to-Launch |
| **B** | `1` | `HOLD` | Zeroes velocity, locks sticks; drone hovers in place |
| **SELECT + B** | `10 + 1` | `GESTURE MODE TOGGLE` | Toggles AI Hand Gesture Control on/off (starts/stops isolated vision process) |
| **L1** | `6` | `OFFBOARD ON` | Enables offboard mode; joystick takes manual control |
| **L2** | `8` | `OFFBOARD OFF` | Disables offboard; flight controller resumes station-keeping |
| **R1** | `7` | `START / RESUME MISSION` | Fresh start: arms, takes off, uploads plan, runs mission. After pause: resumes from the exact paused waypoint |
| **R2** | `9` | `PAUSE MISSION` | Pauses auto mission; enters offboard hover for manual repositioning |
| **SELECT** | `10` | `TELEMETRY` | Logs lat / lon / alt / state to the ROS 2 log |
| **START** | `11` | `⚠ EMERGENCY KILL` | **Immediate motor disarm** — use only in a safety-critical emergency |
| **BTN 13** | `13` | `GEOFENCE ON` | Uploads polygon geofence and enables fence protection |
| **BTN 14** | `14` | `GEOFENCE OFF` | Clears and disables the active geofence |

> **Smart Mission Resume:** **R1** after a pause resumes from the *exact waypoint index* captured at pause time — not from waypoint 0. This is reliable across any number of pause/resume cycles.

---

## 📡 ROS 2 Topics & MAVLink Endpoints

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | Subscribed | Gamepad axes and button states from `joy_node` |
| `/drone_status` | `std_msgs/msg/String` | Published | Mode changes, mission events, kill confirmations |

### MAVLink Endpoint

| Setting | Value |
|---|---|
| Connection string | `udpin://0.0.0.0:14550` |
| Update rate | 20 Hz (`OFFBOARD_HZ`) |
| Compatible FCs | PX4, ArduPilot (SITL or physical) |

---

## 🛡️ Geofencing & Autonomous Mission Setup

### Embedded Polygon Geofence

Configure at the top of [`teleop_node.py`](safe_pilot/teleop_node.py):

| Constant | Default | Description |
|---|---|---|
| `FENCE_ALT_MAX` | `50.0 m` | Altitude ceiling |
| `FENCE_ACTION` | `1` (RTL) | Breach action: `1` = RTL · `2` = Land |
| `GEOFENCE_POLYGON` | `[(lat, lon), …]` | Inclusion polygon vertices |

> **Note:** The coordinates below are **demonstration values only**. Replace them with the actual GPS boundary coordinates of your flight area.

```python
# Close the polygon by repeating the first point
GEOFENCE_POLYGON = [
    (23.176860, 80.022118),   # ← replace with your boundary points
    (23.176497, 80.022207),
    (23.176483, 80.021795),
    (23.176826, 80.021725),
    (23.176860, 80.022118),   # repeated to close loop
]
```

When **BTN 13** is pressed, the node:
1. Clears any existing fence from the FC.
2. Sets `FENCE_TYPE=7`, `FENCE_ENABLE=1`, `FENCE_ACTION`, and `FENCE_ALT_MAX`.
3. Uploads the inclusion polygon via MAVSDK.
4. Monitors the `geofence_result()` stream — on breach, the offboard loop halts so the FC can execute RTL.

---

### Autonomous Waypoint Mission

Add GPS coordinates to `AUTO_WAYPOINTS` in [`teleop_node.py`](safe_pilot/teleop_node.py):

> **Note:** The waypoints, altitude, and speed below are **demonstration values only**. Replace them with the GPS coordinates, altitude, and speed suitable for your actual mission environment.

```python
AUTO_WAYPOINTS = [
    (23.17686, 80.02211),   # WP 0 ← replace with your target coordinates
    (23.17650, 80.02220),   # WP 1
    # … add as many waypoints as needed
]
AUTO_ALTITUDE = 15.0   # Cruise altitude in metres — adjust for your site
AUTO_SPEED    = 5.0    # Flight speed in m/s — adjust to your preference
```

Each waypoint uses `acceptance_radius_m = 2.0` and `is_fly_through = True`. After the last waypoint, **RTL is triggered automatically**.

**Mission flow:**

| Button | Condition | Behaviour |
|---|---|---|
| **R1** | On ground | Arms → takes off to `AUTO_ALTITUDE` → uploads mission → starts |
| **R1** | Already in air | Uploads mission → starts immediately |
| **R1** | Mission paused | Resumes from the exact paused waypoint (no re-upload) |
| **R2** | Mission running | Pauses mission → enters offboard hover |

---

## 🔧 Controller Calibration & Customization

Inspect raw indices from your controller:

```bash
ros2 topic echo /joy
```

Then update the mappings inside `DroneJoyTeleop.__init__` in [`teleop_node.py`](safe_pilot/teleop_node.py):

```python
# ── Axis mapping ───────────────────────────────────────────────────────
self.AXIS_FWD    = 3   # Right stick vertical
self.AXIS_STRAFE = 2   # Right stick horizontal
self.AXIS_THROT  = 1   # Left stick vertical
self.AXIS_YAW    = 0   # Left stick horizontal
self.AXIS_DPAD_V = 7   # D-pad vertical axis
self.AXIS_DPAD_H = 6   # D-pad horizontal axis

# ── Button mapping ─────────────────────────────────────────────────────
self.BTN_ARM_TAKEOFF   = 4    # Y
self.BTN_LAND          = 0    # A
self.BTN_RTL           = 3    # X
self.BTN_HOLD          = 1    # B
self.BTN_START_MISSION = 7    # R1
self.BTN_PAUSE_MISSION = 9    # R2
self.BTN_OFFBOARD_ON   = 6    # L1
self.BTN_OFFBOARD_OFF  = 8    # L2
self.BTN_TELEMETRY     = 10   # SELECT
self.BTN_KILL          = 11   # START
self.BTN_GEOFENCE_ON   = 13
self.BTN_GEOFENCE_OFF  = 14
```

---

## ⚙️ Configuration Reference

### Core Teleoperation Parameters ([`teleop_node.py`](safe_pilot/teleop_node.py))

| Constant | Default | Description |
|---|---|---|
| `DRONE_ADDRESS` | `udpin://0.0.0.0:14550` | MAVLink UDP connection string |
| `TAKEOFF_ALT` | `5.0 m` | Arm + takeoff target altitude |
| `MAX_VEL_XY` | `10.0 m/s` | Maximum horizontal speed cap |
| `MAX_VEL_Z` | `10.0 m/s` | Maximum vertical speed cap |
| `MAX_YAW_RATE` | `60.0 deg/s` | Maximum yaw rate cap |
| `OFFBOARD_HZ` | `20 Hz` | Offboard velocity set-point frequency |
| `DEADBAND` | `0.05` | Joystick axis dead-band threshold |
| `FENCE_ALT_MAX` | `50.0 m` | Geofence altitude ceiling |
| `FENCE_ACTION` | `1` (RTL) | Geofence breach action (`1` = RTL, `2` = Land) |
| `AUTO_ALTITUDE` | `15.0 m` | Autonomous mission cruise altitude |
| `AUTO_SPEED` | `5.0 m/s` | Autonomous mission flight speed |

### Hand Gesture Vision Parameters ([`teleop_gesture_node.py`](safe_pilot/teleop_gesture_node.py))

| Constant | Default | Description |
|---|---|---|
| `GESTURE_CAMERA_ID` | `0` | OpenCV video capture device index |
| `CALIB_FRAMES` | `60` | Calibration frames needed to lock baseline (~2.5–3.0 s) |
| `GESTURE_SENSITIVITY_XY` | `5.0` | Horizontal/vertical translational speed gain (m/s per unit) |
| `GESTURE_SENSITIVITY_Z` | `3.0` | Vertical climb/descend speed gain (m/s per unit) |
| `GESTURE_SENSITIVITY_FWD` | `6.5` | Forward/reverse linear depth speed gain (m/s per scale delta) |
| `GESTURE_SENSITIVITY_YAW` | `75.0` | In-place yaw rotation rate gain (deg/s per unit, fist mode) |
| `GESTURE_MAX_VEL` | `3.0 m/s` | Hard safety ceiling on gesture linear velocity |
| `GESTURE_MAX_YAW_RATE` | `45.0 deg/s` | Hard safety ceiling on gesture yaw rotation rate |
| `GESTURE_HOLD_TIMEOUT` | `1.5 s` | Time without hand detection before triggering automated `DRONE HOLD` |
| `EMA_ALPHA` | `0.30` | Exponential moving average smoothing factor for landmark stability |

---

## 📄 License

See [`package.xml`](package.xml) for maintainer and license details.

---

<div align="center">

**SafePilot** — Because crashing is optional.

*Built with ROS 2 · MAVSDK · ArduPilot · Gazebo Harmonic*

*Maintainer: **abhinav** · `ironman18122004@gmail.com`*

</div>
