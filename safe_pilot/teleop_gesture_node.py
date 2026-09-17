#!/usr/bin/env python3
"""
SafePilot — Semi-Autonomous Drone Teleop Node  (Gesture Edition - Multi-Core)
=============================================================================
High-performance drone teleoperation combining standard gamepad joystick control
with real-time AI hand gesture control powered by MediaPipe and OpenCV.

Multi-Core Architecture:
------------------------
• Dedicated Process 1: ROS 2 Teleop Node & MAVSDK Controller
  – Runs rclpy executor in main thread.
  – MAVSDK 20 Hz offboard setpoint loop in a dedicated background asyncio thread.
  – Non-blocking ROS 2 timer (50 Hz) ingests gesture commands from IPC queue
    strictly inside the ROS executor thread context, eliminating DDS multi-thread
    segmentation faults.
• Dedicated Process 2: Gesture Vision Engine (pinned to dedicated CPU cores)
  – Separate OS process running MediaPipe HandLandmarker Tasks API.
  – Direct camera capture with minimal latency (CAP_PROP_BUFFERSIZE=1).
  – Real-time HUD visualizer & OpenCV window.
  – Communicates control setpoints to ROS 2 over a lightweight lock-free IPC queue.

Hand Gesture Control:
---------------------
Toggle:  SELECT (btn 10) + B (btn 1) pressed simultaneously.

While gesture mode is ACTIVE:
  • All joystick axes and normal buttons are DISABLED (except SELECT+B toggle).
  • Calibration phase: hold hand still inside the guide circle (~3 seconds).
    Neutral reference point and bounding box area lock automatically.
  • Open Hand Mode (Translational Flight):
      – Hand moves UP        → drone climbs (negative Z velocity in NED)
      – Hand moves DOWN      → drone descends (positive Z velocity in NED)
      – Hand moves LEFT      → drone strafes left (-Y velocity)
      – Hand moves RIGHT     → drone strafes right (+Y velocity)
      – Hand moves CLOSER    → drone moves FORWARD (+X velocity)
      – Hand moves FARTHER   → drone moves BACKWARD (-X velocity)
  • Closed Fist Mode (Yaw Rotation Mode):
      – Close hand into a FIST to activate in-place yaw rotation
      – Fist moves RIGHT     → drone yaws clockwise (+yaw rate)
      – Fist moves LEFT      → drone yaws counter-clockwise (-yaw rate)
      – Fist moves UP/DOWN   → drone climbs/descends (vz)
      – Strafe (vy) and Forward/Back (vx) are locked to 0.0 to prevent drift
  • Fail-safe protection:
      – If hand is lost > 1.5 s, drone automatically enters HOLD mode.
      – Upon hand return, automatic re-calibration occurs before restoring velocity.
"""

import os
import sys
import time
import math
import signal
import asyncio
import threading
import traceback
import multiprocessing as mp
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# MAVSDK imports
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.action import ActionError
from mavsdk.geofence import Point, Polygon, FenceType, GeofenceData
from mavsdk.mission import MissionItem, MissionPlan


# ===============================================================================
#  Configuration
# ===============================================================================
DRONE_ADDRESS   = "udpin://0.0.0.0:14550"
TAKEOFF_ALT     = 5.0
MAX_VEL_XY      = 10.0
MAX_VEL_Z       = 10.0
MAX_YAW_RATE    = 60.0
OFFBOARD_HZ     = 20
DEADBAND        = 0.05

# Geofence
FENCE_ALT_MAX   = 50.0
FENCE_ACTION    = 1

GEOFENCE_POLYGON = [
    (-35.36324697, 149.16615638),
    (-35.36340159, 149.16514499),
    (-35.36253012, 149.16494645),
    (-35.36237550, 149.16595783),
    (-35.36324697, 149.16615638),
]

# Auto Mission
AUTO_WAYPOINTS = [
    (-35.36374855, 149.16611902),
    (-35.36358659, 149.16609438),
    (-35.36358659, 149.16496051),
    (-35.36342463, 149.16495900),
    (-35.36342463, 149.16606975),
    (-35.36326267, 149.16604512),
    (-35.36326267, 149.16495750),
    (-35.36310071, 149.16495599),
    (-35.36310071, 149.16602048),
    (-35.36293875, 149.16599585),
    (-35.36293875, 149.16495449),
    (-35.36277679, 149.16495298),
    (-35.36277679, 149.16597122),
    (-35.36261483, 149.16594658),
    (-35.36261483, 149.16495148),
    (-35.36245287, 149.16494997),
    (-35.36245287, 149.16592195),
]
AUTO_ALTITUDE   = 15.0
AUTO_SPEED      = 5.0

# Hand Gesture Configuration
GESTURE_CAMERA_ID       = 0       # OpenCV camera index
CALIB_FRAMES            = 60      # Frames needed to lock calibration (~2.5-3.0s at 20-25fps)
GESTURE_SENSITIVITY_XY  = 5.0     # m/s per normalized unit
GESTURE_SENSITIVITY_Z   = 3.0     # m/s per normalized vertical displacement
GESTURE_SENSITIVITY_FWD = 6.5     # m/s per normalized linear depth scale delta
GESTURE_SENSITIVITY_YAW = 75.0    # deg/s per normalized horizontal displacement unit (fist mode)
GESTURE_HOLD_TIMEOUT    = 1.5     # seconds without detection before triggering HOLD
GESTURE_MAX_VEL         = 3.0     # Hard safety clamp on gesture linear velocity (m/s)
GESTURE_MAX_YAW_RATE    = 45.0    # Hard safety clamp on gesture yaw rate (deg/s)

# MediaPipe fingertip landmark indices (Thumb, Index, Middle, Ring, Pinky)
FINGERTIP_IDS = [4, 8, 12, 16, 20]

# Model path
HAND_LANDMARKER_MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task"
)


# ===============================================================================
#  GestureVisionProcess — Dedicated High-Performance Vision Subprocess
# ===============================================================================
class GestureVisionProcess(mp.Process):
    """
    Independent OS process executing MediaPipe inference, state tracking, and HUD
    rendering on dedicated CPU cores. Sends sanitized control tuples to ROS 2.
    """

    def __init__(self, cmd_queue: mp.Queue, stop_event: mp.Event, target_cores: Optional[list] = None):
        super().__init__(daemon=False, name="SafePilotGestureVision")
        self.cmd_queue = cmd_queue
        self.stop_event = stop_event
        self.target_cores = target_cores

    def run(self):
        # Ignore SIGINT in child so ROS 2 node handles clean parent shutdown
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # 1. Pin to dedicated CPU cores if available
        if self.target_cores:
            try:
                os.sched_setaffinity(0, set(self.target_cores))
                print(f"[GestureVision] Process (PID {os.getpid()}) pinned to CPU cores: {self.target_cores}")
            except Exception as e:
                print(f"[GestureVision] Could not set CPU affinity: {e}")

        # 2. Local imports to avoid overhead in parent ROS process
        import cv2
        import numpy as np
        import mediapipe as mp_lib
        from mediapipe.tasks.python import vision as mp_vision
        from mediapipe.tasks.python.core import base_options as mp_base

        if not os.path.isfile(HAND_LANDMARKER_MODEL):
            print(f"[GestureVision] ERROR: Model not found at {HAND_LANDMARKER_MODEL}")
            try:
                self.cmd_queue.put_nowait(('hold',))
            except Exception:
                pass
            return

        # 3. Initialize Camera
        cap = cv2.VideoCapture(GESTURE_CAMERA_ID)
        if not cap.isOpened():
            # Try V4L2 fallback if default fails
            cap = cv2.VideoCapture(GESTURE_CAMERA_ID, cv2.CAP_V4L2)

        if not cap.isOpened():
            print(f"[GestureVision] ERROR: Cannot open video capture device {GESTURE_CAMERA_ID}")
            try:
                self.cmd_queue.put_nowait(('hold',))
            except Exception:
                pass
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 4. Initialize MediaPipe HandLandmarker
        base_opts = mp_base.BaseOptions(model_asset_path=HAND_LANDMARKER_MODEL)
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_opts,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        hand_connections = mp_vision.HandLandmarksConnections.HAND_CONNECTIONS

        # State machine variables
        STATE_CALIBRATING = 'CALIBRATING'
        STATE_ACTIVE      = 'ACTIVE'
        STATE_LOST        = 'LOST'
        EMA_ALPHA         = 0.35

        state         = STATE_CALIBRATING
        calib_counter = 0
        ref_cx = ref_cy = ref_area = None
        ema_cx = ema_cy = ema_area = None
        last_hand_time = time.time()
        frame_ts_ms    = 0

        def clamp_vel(v: float) -> float:
            return max(-GESTURE_MAX_VEL, min(GESTURE_MAX_VEL, v))

        def clamp_yaw(v: float) -> float:
            return max(-GESTURE_MAX_YAW_RATE, min(GESTURE_MAX_YAW_RATE, v))

        def is_closed_fist(lms) -> bool:
            wrist = lms[0]
            curled_count = 0
            # Check 4 main fingers: (tip, pip) -> index(8,6), middle(12,10), ring(16,14), pinky(20,18)
            for tip_idx, pip_idx in ((8, 6), (12, 10), (16, 14), (20, 18)):
                t = lms[tip_idx]
                p = lms[pip_idx]
                d_tip = math.hypot(t.x - wrist.x, t.y - wrist.y, t.z - wrist.z)
                d_pip = math.hypot(p.x - wrist.x, p.y - wrist.y, p.z - wrist.z)
                if d_tip < d_pip:
                    curled_count += 1
            # Index and middle must both be curled, plus at least 3 fingers total
            idx_curled = math.hypot(lms[8].x - wrist.x, lms[8].y - wrist.y, lms[8].z - wrist.z) < \
                         math.hypot(lms[6].x - wrist.x, lms[6].y - wrist.y, lms[6].z - wrist.z)
            mid_curled = math.hypot(lms[12].x - wrist.x, lms[12].y - wrist.y, lms[12].z - wrist.z) < \
                         math.hypot(lms[10].x - wrist.x, lms[10].y - wrist.y, lms[10].z - wrist.z)
            return (curled_count >= 3) and idx_curled and mid_curled

        window_name = "SafePilot -- Hand Gesture Control"
        has_display = True
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 640, 480)
        except Exception as e:
            print(f"[GestureVision] Display window initialization notice: {e}. Running headlessly.")
            has_display = False

        print("[GestureVision] Vision pipeline running. Ready for gestures.")

        shown_frames = 0
        consecutive_fail_reads = 0
        fist_debounce = 0
        is_fist = False

        try:
            with mp_vision.HandLandmarker.create_from_options(options) as landmarker:
                while not self.stop_event.is_set():
                    ret, raw = cap.read()
                    if self.stop_event.is_set():
                        break
                    if not ret or raw is None:
                        consecutive_fail_reads += 1
                        if consecutive_fail_reads > 30:
                            self._send_cmd(('hold',))
                        time.sleep(0.01)
                        continue
                    consecutive_fail_reads = 0

                    # Mirror horizontal flip for natural intuitive interaction
                    raw = cv2.flip(raw, 1)
                    h, w = raw.shape[:2]
                    frame = raw.copy()

                    # Translucent dark header bar for crisp HUD contrast (68px high, 2 rows)
                    header_overlay = frame[:68, :].copy()
                    cv2.rectangle(frame, (0, 0), (w, 68), (15, 18, 22), -1)
                    cv2.addWeighted(frame[:68, :], 0.70, header_overlay, 0.30, 0, frame[:68, :])

                    # Convert color for MediaPipe
                    rgb = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
                    mp_img = mp_lib.Image(image_format=mp_lib.ImageFormat.SRGB, data=rgb)
                    
                    cur_ts_ms = int(time.time() * 1000)
                    if cur_ts_ms <= frame_ts_ms:
                        cur_ts_ms = frame_ts_ms + 1
                    frame_ts_ms = cur_ts_ms

                    result = landmarker.detect_for_video(mp_img, frame_ts_ms)

                    hand_detected = (
                        result.hand_landmarks is not None
                        and len(result.hand_landmarks) > 0
                    )

                    # Guide circle (neutral target region)
                    cx_g = w // 2
                    cy_g = h // 2
                    r_g  = min(w, h) // 5
                    cv2.circle(frame, (cx_g, cy_g), r_g, (60, 180, 70), 2)

                    if hand_detected:
                        lms = result.hand_landmarks[0]

                        # Filter valid non-NaN points
                        valid_lms = [lm for lm in lms if not (math.isnan(lm.x) or math.isnan(lm.y))]
                        if len(valid_lms) < 21:
                            hand_detected = False

                    if hand_detected:
                        lms = result.hand_landmarks[0]

                        # Draw skeleton landmarks & connections
                        pts = [(int(lm.x * w), int(lm.y * h)) for lm in lms]
                        for conn in hand_connections:
                            s, e = conn.start, conn.end
                            if s < len(pts) and e < len(pts):
                                cv2.line(frame, pts[s], pts[e], (0, 220, 100), 2)
                        for px, py in pts:
                            cv2.circle(frame, (px, py), 3, (255, 255, 255), -1)

                        # Check if hand is curled into a closed fist
                        raw_fist = is_closed_fist(lms)
                        if raw_fist:
                            fist_debounce = min(10, fist_debounce + 1)
                        else:
                            fist_debounce = max(0, fist_debounce - 1)
                        is_fist = (fist_debounce >= 3)

                        # Calculate hand center: use knuckles when fist is closed for rock-solid stability
                        if is_fist:
                            mcp_xs = [lms[i].x for i in (5, 9, 13, 17) if not math.isnan(lms[i].x)]
                            mcp_ys = [lms[i].y for i in (5, 9, 13, 17) if not math.isnan(lms[i].y)]
                            raw_cx = float(sum(mcp_xs) / len(mcp_xs)) if mcp_xs else 0.5
                            raw_cy = float(sum(mcp_ys) / len(mcp_ys)) if mcp_ys else 0.5
                        else:
                            tip_xs = [lms[i].x for i in FINGERTIP_IDS if not math.isnan(lms[i].x)]
                            tip_ys = [lms[i].y for i in FINGERTIP_IDS if not math.isnan(lms[i].y)]
                            raw_cx = float(sum(tip_xs) / len(tip_xs)) if tip_xs else 0.5
                            raw_cy = float(sum(tip_ys) / len(tip_ys)) if tip_ys else 0.5

                        # Bounding box area for depth proxy
                        all_xs = [lm.x for lm in lms if not math.isnan(lm.x)]
                        all_ys = [lm.y for lm in lms if not math.isnan(lm.y)]
                        raw_area = float((max(all_xs) - min(all_xs)) * (max(all_ys) - min(all_ys))) if all_xs and all_ys else 0.05

                        # Exponential Moving Average (EMA) smoothing
                        if ema_cx is None:
                            ema_cx, ema_cy, ema_area = raw_cx, raw_cy, raw_area
                        else:
                            ema_cx   = EMA_ALPHA * raw_cx   + (1.0 - EMA_ALPHA) * ema_cx
                            ema_cy   = EMA_ALPHA * raw_cy   + (1.0 - EMA_ALPHA) * ema_cy
                            ema_area = EMA_ALPHA * raw_area + (1.0 - EMA_ALPHA) * ema_area

                        cur_px = int(ema_cx * w)
                        cur_py = int(ema_cy * h)
                        pt_color = (0, 220, 255) if is_fist else (0, 255, 255)
                        ring_color = (255, 140, 0) if is_fist else (255, 200, 0)
                        cv2.circle(frame, (cur_px, cur_py), 8, pt_color, -1)
                        cv2.circle(frame, (cur_px, cur_py), 12, ring_color, 2)
                        last_hand_time = time.time()

                        # ── State Machine ──
                        if state == STATE_LOST:
                            state = STATE_CALIBRATING
                            calib_counter = 0
                            ema_cx = ema_cy = ema_area = None
                            ref_cx = ref_cy = ref_area = None
                            self._send_cmd(('recalibrating',))

                        elif state == STATE_CALIBRATING:
                            if is_fist:
                                # Row 1: Mode badge and prompt
                                cv2.putText(frame, "[ CALIBRATING: OPEN HAND ]", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 160, 255), 2, cv2.LINE_AA)
                                r_text = "HOLD STILL"
                                tw = cv2.getTextSize(r_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                                cv2.putText(frame, r_text, (w - tw - 15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1, cv2.LINE_AA)
                                # Row 2: Prompt
                                cv2.putText(frame, "Open hand flat in guide circle to calibrate", (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1, cv2.LINE_AA)
                            else:
                                calib_counter += 1
                                prog = min(calib_counter / CALIB_FRAMES, 1.0)
                                sec_left = max(0, int((CALIB_FRAMES - calib_counter) / 20) + 1)

                                # Animated radial progress gauge
                                cv2.ellipse(
                                    frame, (cx_g, cy_g), (r_g + 8, r_g + 8),
                                    -90, 0, int(360 * prog), (0, 215, 255), 4
                                )
                                # Row 1
                                cv2.putText(frame, "[ CALIBRATING... ]", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 215, 255), 2, cv2.LINE_AA)
                                r_text = f"LOCKING IN {sec_left}s"
                                tw = cv2.getTextSize(r_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                                cv2.putText(frame, r_text, (w - tw - 15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 215, 255), 1, cv2.LINE_AA)
                                # Row 2
                                cv2.putText(frame, f"Hold hand steady inside green circle ({int(prog*100)}%)", (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 235, 255), 1, cv2.LINE_AA)
                                self._send_cmd(('calibrating',))

                                if calib_counter >= CALIB_FRAMES:
                                    ref_cx, ref_cy, ref_area = ema_cx, ema_cy, ema_area
                                    state = STATE_ACTIVE
                                    print(f"[GestureVision] Calibration LOCKED: ref_cx={ref_cx:.3f}, ref_cy={ref_cy:.3f}, ref_area={ref_area:.4f}")

                        elif state == STATE_ACTIVE:
                            dx = ema_cx   - ref_cx
                            dy = ema_cy   - ref_cy

                            # Linear depth proxy (relative scale vs calibration reference)
                            curr_size = math.sqrt(max(0.001, ema_area))
                            ref_size  = math.sqrt(max(0.001, ref_area))
                            delta_scale = (curr_size - ref_size) / ref_size

                            # Deadband on normalized gestures
                            if abs(dx) < 0.03:
                                dx = 0.0
                            if abs(dy) < 0.03:
                                dy = 0.0
                            if abs(delta_scale) < 0.05:
                                delta_scale = 0.0

                            if is_fist:
                                # ── FIST MODE: Left/Right translates to YAW rotation ──
                                vx  = 0.0   # Lock forward/back to prevent false backward drift
                                vy  = 0.0   # Lock strafe to zero during yaw
                                vz  = clamp_vel(dy * GESTURE_SENSITIVITY_Z)
                                yaw = clamp_yaw(dx * GESTURE_SENSITIVITY_YAW)

                                self._send_cmd(('vel', vx, vy, vz, yaw))

                                # Row 1: Mode badge and Hand state
                                cv2.putText(frame, "[ MODE: YAW ROTATION ]", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2, cv2.LINE_AA)
                                r_text = "CLOSED FIST"
                                tw = cv2.getTextSize(r_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                                cv2.putText(frame, r_text, (w - tw - 15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 240, 255), 1, cv2.LINE_AA)

                                # Row 2: Clean telemetry
                                cv2.putText(frame, f"YAW: {yaw:+5.1f} deg/s   VZ: {vz:+4.1f} m/s   (STRAFE LOCKED)",
                                            (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 255), 2, cv2.LINE_AA)

                                # Directional rotation indicators
                                if abs(yaw) > 1.0:
                                    if yaw > 0:
                                        cv2.ellipse(frame, (cx_g, cy_g), (r_g + 18, r_g + 18), 0, -30, 60, (0, 220, 255), 3)
                                        cv2.putText(frame, ">> YAW RIGHT >>", (cx_g - 65, cy_g + r_g + 35),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
                                    else:
                                        cv2.ellipse(frame, (cx_g, cy_g), (r_g + 18, r_g + 18), 0, 120, 210, (0, 220, 255), 3)
                                        cv2.putText(frame, "<< YAW LEFT <<", (cx_g - 60, cy_g + r_g + 35),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
                                else:
                                    cv2.putText(frame, "YAW NEUTRAL (MOVE LEFT/RIGHT)", (cx_g - 120, cy_g + r_g + 35),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (160, 200, 220), 1, cv2.LINE_AA)

                            else:
                                # ── OPEN HAND MODE: Standard Translational Strafe/Climb/Forward ──
                                vx  = clamp_vel(delta_scale * GESTURE_SENSITIVITY_FWD)
                                vy  = clamp_vel(dx * GESTURE_SENSITIVITY_XY)
                                vz  = clamp_vel(dy * GESTURE_SENSITIVITY_Z)
                                yaw = 0.0

                                self._send_cmd(('vel', vx, vy, vz, yaw))

                                # Row 1: Mode badge and Hand state
                                cv2.putText(frame, "[ MODE: STRAFE ]", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 100), 2, cv2.LINE_AA)
                                r_text = "OPEN HAND"
                                tw = cv2.getTextSize(r_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                                cv2.putText(frame, r_text, (w - tw - 15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 255, 180), 1, cv2.LINE_AA)

                                # Row 2: Clean telemetry with FWD/REV direction tags
                                fwd_hint = " (FWD)" if vx > 0.25 else (" (REV)" if vx < -0.25 else "")
                                cv2.putText(frame, f"VX: {vx:+4.1f}{fwd_hint}   VY: {vy:+4.1f}   VZ: {vz:+4.1f} m/s",
                                            (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 2, cv2.LINE_AA)

                                # Dynamic visual depth feedback circle (expanding green for forward, contracting orange for reverse)
                                if abs(delta_scale) > 0.03:
                                    depth_r = max(10, int(r_g * max(0.4, min(1.8, 1.0 + delta_scale))))
                                    depth_col = (0, 255, 160) if delta_scale > 0 else (0, 180, 255)
                                    cv2.circle(frame, (cx_g, cy_g), depth_r, depth_col, 2, cv2.LINE_AA)

                                # Vector line from center guide to current hand displacement
                                end_px = cx_g + int(dx * w * 0.8)
                                end_py = cy_g + int(dy * h * 0.8)
                                if abs(dx) > 0.01 or abs(dy) > 0.01:
                                    cv2.arrowedLine(
                                        frame, (cx_g, cy_g),
                                        (end_px, end_py),
                                        (0, 255, 150), 3, tipLength=0.25
                                    )

                    else:
                        # No hand detected
                        ema_cx = ema_cy = ema_area = None
                        if time.time() - last_hand_time > GESTURE_HOLD_TIMEOUT:
                            if state != STATE_LOST:
                                state = STATE_LOST
                                self._send_cmd(('hold',))
                                print("[GestureVision] Hand lost. Triggered drone HOLD.")

                        # Row 1: Status badge & Drone state
                        cv2.putText(frame, "[ STATUS: NO HAND DETECTED ]", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 100, 255), 2, cv2.LINE_AA)
                        r_text = "DRONE IN HOLD"
                        tw = cv2.getTextSize(r_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
                        cv2.putText(frame, r_text, (w - tw - 15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1, cv2.LINE_AA)

                        # Row 2: Guide instruction
                        cv2.putText(frame, "Show hand inside green circle to take control", (15, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 220), 1, cv2.LINE_AA)

                    # Translucent footer bar (32px tall)
                    footer_overlay = frame[h - 32:h, :].copy()
                    cv2.rectangle(frame, (0, h - 32), (w, h), (15, 18, 22), -1)
                    cv2.addWeighted(frame[h - 32:h, :], 0.75, footer_overlay, 0.25, 0, frame[h - 32:h, :])
                    cv2.putText(frame, "SafePilot Gesture Control (Multi-Core)", (15, h - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)
                    exit_hint = "SELECT+B or 'Q' to exit"
                    tw_exit = cv2.getTextSize(exit_hint, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
                    cv2.putText(frame, exit_hint, (w - tw_exit - 15, h - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv2.LINE_AA)

                    # Display frame in OpenCV window
                    if has_display:
                        try:
                            cv2.imshow(window_name, frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                self._send_cmd(('exit',))
                                self.stop_event.set()
                                break

                            # Clean exit if user closes window via window manager 'X' button
                            shown_frames += 1
                            if shown_frames > 20:
                                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                                    print("[GestureVision] OpenCV window closed by user. Stopping gesture control.")
                                    self._send_cmd(('exit',))
                                    self.stop_event.set()
                                    break
                        except Exception as e:
                            print(f"[GestureVision] GUI display notice: {e}. Continuing headlessly.")
                            has_display = False

        except Exception as e:
            print(f"[GestureVision] Exception in vision loop: {e}\n{traceback.format_exc()}")
        finally:
            cap.release()
            if has_display:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            print("[GestureVision] Process cleanly terminated.")
            os._exit(0)

    def _send_cmd(self, item: tuple):
        """Thread-safe, non-blocking queue put with drop-oldest behavior if full."""
        try:
            self.cmd_queue.put_nowait(item)
        except Exception:
            # Queue is full, pop oldest and push newest
            try:
                self.cmd_queue.get_nowait()
                self.cmd_queue.put_nowait(item)
            except Exception:
                pass


# ===============================================================================
#  DroneController — MAVSDK Interface
# ===============================================================================
class DroneController:
    """
    Wraps MAVSDK System with thread-safe getters and coroutine callers.
    Runs on an independent asyncio event loop in a dedicated background thread.
    """

    def __init__(self):
        self.drone            = System()
        self.connected        = False
        self.armed            = False
        self.in_air           = False
        self.offboard_on      = False
        self.mission_active   = False
        self.mission_paused   = False
        self.mission_uploaded = False
        self.is_holding       = False

        self._current_mission_item = 0
        self._paused_mission_item  = 0

        self._vx  = 0.0
        self._vy  = 0.0
        self._vz  = 0.0
        self._yaw = 0.0
        self._lock = threading.Lock()

        self.telem_lat     = 0.0
        self.telem_lon     = 0.0
        self.telem_alt     = 0.0
        self.telem_heading = 0.0

        self.loop: Optional[asyncio.AbstractEventLoop] = None

    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float = 0.0):
        with self._lock:
            self._vx  = float(vx)
            self._vy  = float(vy)
            self._vz  = float(vz)
            self._yaw = float(yaw)

    def get_velocity(self) -> Tuple[float, float, float, float]:
        with self._lock:
            return self._vx, self._vy, self._vz, self._yaw

    async def run(self):
        print(f"[SafePilot] Connecting to drone at {DRONE_ADDRESS} ...")
        await self.drone.connect(system_address=DRONE_ADDRESS)
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True
                print("[SafePilot] Connected to drone via MAVLink!")
                break

        # Start continuous telemetry & control streams
        asyncio.ensure_future(self._stream_position())
        asyncio.ensure_future(self._stream_armed())
        asyncio.ensure_future(self._stream_in_air())
        asyncio.ensure_future(self._stream_geofence_status())
        asyncio.ensure_future(self._stream_mission_progress())
        asyncio.ensure_future(self._offboard_loop())

        print("[SafePilot] Telemetry & offboard loops online. Ready for control.")
        while True:
            await asyncio.sleep(1.0)

    async def _stream_position(self):
        async for pos in self.drone.telemetry.position():
            self.telem_lat = pos.latitude_deg
            self.telem_lon = pos.longitude_deg
            self.telem_alt = pos.relative_altitude_m

    async def _stream_armed(self):
        async for is_armed in self.drone.telemetry.armed():
            self.armed = is_armed

    async def _stream_in_air(self):
        async for air in self.drone.telemetry.in_air():
            self.in_air = air

    async def _stream_geofence_status(self):
        try:
            async for result in self.drone.geofence.geofence_result():
                if hasattr(result, 'is_breached') and result.is_breached:
                    print("[SafePilot] GEOFENCE BREACH! Disabling offboard for RTL.")
                    self.offboard_on = False
        except Exception:
            pass

    async def _stream_mission_progress(self):
        try:
            async for progress in self.drone.mission.mission_progress():
                self._current_mission_item = progress.current
                if (self.mission_active
                        and progress.total > 0
                        and progress.current >= progress.total):
                    print("[SafePilot] Mission complete! Initiating RTL ...")
                    self.mission_active = False
                    self.mission_paused = False
                    asyncio.ensure_future(self.cmd_rtl())
        except Exception:
            pass

    async def _offboard_loop(self):
        dt = 1.0 / OFFBOARD_HZ
        while True:
            if self.offboard_on:
                vx, vy, vz, yaw = self.get_velocity()
                try:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(vx, vy, vz, yaw)
                    )
                except OffboardError as e:
                    print(f"[SafePilot] Offboard loop error: {e}")
            await asyncio.sleep(dt)

    async def cmd_arm_takeoff(self, altitude=TAKEOFF_ALT):
        if not self.connected:
            print("[SafePilot] Drone not connected — cannot arm/takeoff.")
            return
        try:
            print("[SafePilot] Arming ...")
            await self.drone.action.arm()
            print(f"[SafePilot] Taking off to {altitude:.1f} m ...")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()
            async for pos in self.drone.telemetry.position():
                if pos.relative_altitude_m >= altitude * 0.95:
                    print("[SafePilot] Takeoff altitude reached.")
                    break
            await self._start_offboard()
        except ActionError as e:
            print(f"[SafePilot] Arm/Takeoff error: {e}")

    async def cmd_land(self):
        try:
            print("[SafePilot] Landing ...")
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.action.land()
        except ActionError as e:
            print(f"[SafePilot] Land error: {e}")

    async def cmd_rtl(self):
        try:
            print("[SafePilot] Return-to-Launch (RTL) ...")
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.action.return_to_launch()
        except ActionError as e:
            print(f"[SafePilot] RTL error: {e}")

    async def cmd_hold(self):
        print("[SafePilot] HOLD active — zeroing velocity.")
        self.is_holding = True
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
        if not self.offboard_on:
            await self._start_offboard()

    async def cmd_start_mission(self):
        try:
            if not AUTO_WAYPOINTS:
                print("[SafePilot] AUTO_WAYPOINTS empty!")
                return
            if not self.in_air:
                if not self.connected:
                    print("[SafePilot] Not connected.")
                    return
                if self.offboard_on:
                    await self._stop_offboard()
                await self.drone.action.arm()
                await self.drone.action.set_takeoff_altitude(AUTO_ALTITUDE)
                await self.drone.action.takeoff()
                async for pos in self.drone.telemetry.position():
                    if pos.relative_altitude_m >= AUTO_ALTITUDE * 0.95:
                        break
            else:
                if self.offboard_on:
                    await self._stop_offboard()

            mission_items = []
            for lat, lon in AUTO_WAYPOINTS:
                mission_items.append(MissionItem(
                    latitude_deg=lat,
                    longitude_deg=lon,
                    relative_altitude_m=AUTO_ALTITUDE,
                    speed_m_s=AUTO_SPEED,
                    is_fly_through=True,
                    gimbal_pitch_deg=float('nan'),
                    gimbal_yaw_deg=float('nan'),
                    camera_action=MissionItem.CameraAction.NONE,
                    loiter_time_s=float('nan'),
                    camera_photo_interval_s=float('nan'),
                    camera_photo_distance_m=float('nan'),
                    acceptance_radius_m=2.0,
                    yaw_deg=float('nan'),
                    vehicle_action=MissionItem.VehicleAction.NONE,
                ))
            await self.drone.mission.upload_mission(MissionPlan(mission_items))
            await self.drone.mission.set_return_to_launch_after_mission(True)
            self.mission_uploaded = True
            await self.drone.mission.start_mission()
            self.mission_active = True
            self.mission_paused = False
        except Exception as e:
            print(f"[SafePilot] Mission start error: {e}")

    async def cmd_resume_mission(self):
        try:
            if not self.mission_uploaded:
                await self.cmd_start_mission()
                return
            resume_idx = self._paused_mission_item
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.mission.set_current_mission_item(resume_idx)
            await self.drone.mission.start_mission()
            self.mission_active = True
            self.mission_paused = False
        except Exception as e:
            print(f"[SafePilot] Mission resume error: {e}")

    async def cmd_pause_mission(self):
        try:
            self._paused_mission_item = self._current_mission_item
            await self.drone.mission.pause_mission()
            self.mission_active = False
            self.mission_paused = True
            await self._start_offboard()
        except Exception as e:
            print(f"[SafePilot] Mission pause error: {e}")

    async def cmd_enable_offboard(self):
        self.is_holding = False
        if not self.offboard_on:
            await self._start_offboard()

    async def cmd_disable_offboard(self):
        if self.offboard_on:
            self.set_velocity(0.0, 0.0, 0.0, 0.0)
            await asyncio.sleep(0.3)
            await self._stop_offboard()

    async def cmd_kill(self):
        print("[SafePilot] EMERGENCY KILL TRIGGERED!")
        try:
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.action.kill()
        except ActionError as e:
            print(f"[SafePilot] Kill error: {e}")

    async def cmd_enable_geofence(self):
        try:
            await self.drone.geofence.clear_geofence()
            await self.drone.param.set_param_int("FENCE_TYPE", 7)
            await self.drone.param.set_param_int("FENCE_ENABLE", 1)
            await self.drone.param.set_param_int("FENCE_ACTION", FENCE_ACTION)
            await self.drone.param.set_param_float("FENCE_ALT_MAX", float(FENCE_ALT_MAX))
            if GEOFENCE_POLYGON:
                points  = [Point(lat, lon) for lat, lon in GEOFENCE_POLYGON]
                polygon = Polygon(points, FenceType.INCLUSION)
                await self.drone.geofence.upload_geofence(GeofenceData([polygon], []))
                print(f"[SafePilot] Geofence uploaded & enabled (Max alt: {FENCE_ALT_MAX}m)")
        except Exception as e:
            print(f"[SafePilot] Enable geofence error: {e}")

    async def cmd_disable_geofence(self):
        try:
            await self.drone.param.set_param_int("FENCE_ENABLE", 0)
            await self.drone.geofence.clear_geofence()
            print("[SafePilot] Geofence DISABLED")
        except Exception as e:
            print(f"[SafePilot] Disable geofence error: {e}")

    async def _start_offboard(self):
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
        try:
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await self.drone.offboard.start()
            self.offboard_on = True
            print("[SafePilot] Offboard mode ON.")
        except OffboardError as e:
            print(f"[SafePilot] Offboard start error: {e}")

    async def _stop_offboard(self):
        try:
            await self.drone.offboard.stop()
            self.offboard_on = False
            print("[SafePilot] Offboard mode OFF.")
        except OffboardError as e:
            print(f"[SafePilot] Offboard stop error: {e}")

    def get_telemetry_str(self) -> str:
        return (
            f"Lat={self.telem_lat:.7f} | "
            f"Lon={self.telem_lon:.7f} | "
            f"Alt={self.telem_alt:.2f}m | "
            f"Armed={self.armed} | InAir={self.in_air} | "
            f"Offboard={self.offboard_on} | Holding={self.is_holding}"
        )


# ===============================================================================
#  DroneJoyTeleop — ROS 2 Node
# ===============================================================================
class DroneJoyTeleop(Node):
    """
    ROS 2 node handling joystick teleoperation, gesture mode toggling, and
    thread-safe IPC polling via a high-rate ROS 2 timer.
    """

    def __init__(self, drone_ctrl: DroneController):
        super().__init__('safe_pilot_teleop_gesture')
        self.dc = drone_ctrl

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.status_pub = self.create_publisher(String, '/drone_status', 10)

        # 50 Hz ROS Timer to poll IPC queue safely inside ROS executor thread
        self.gesture_timer = self.create_timer(0.02, self._gesture_poll_callback)

        # Axis mapping
        self.AXIS_FWD    = 3
        self.AXIS_STRAFE = 2
        self.AXIS_THROT  = 1
        self.AXIS_YAW    = 0
        self.AXIS_DPAD_V = 7
        self.AXIS_DPAD_H = 6

        # Button mapping
        self.BTN_ARM_TAKEOFF   = 4
        self.BTN_LAND          = 0
        self.BTN_RTL           = 3
        self.BTN_HOLD          = 1    # B — also part of gesture toggle combo
        self.BTN_START_MISSION = 7
        self.BTN_PAUSE_MISSION = 9
        self.BTN_OFFBOARD_ON   = 6
        self.BTN_OFFBOARD_OFF  = 8
        self.BTN_TELEMETRY     = 10   # SELECT — also part of gesture toggle combo
        self.BTN_KILL          = 11
        self.BTN_GEOFENCE_ON   = 13
        self.BTN_GEOFENCE_OFF  = 14

        # Speed scaling
        self.vel_scale_xy = 2.0
        self.vel_scale_z  = 1.0
        self.yaw_scale    = 30.0

        self.VEL_XY_MIN = 0.2
        self.VEL_XY_MAX = MAX_VEL_XY
        self.VEL_Z_MIN  = 0.2
        self.VEL_Z_MAX  = MAX_VEL_Z
        self.YAW_MIN    = 5.0
        self.YAW_MAX    = MAX_YAW_RATE

        self._prev = {}

        # Gesture mode management
        self.gesture_mode = False
        self._gesture_proc: Optional[GestureVisionProcess] = None
        self._gesture_stop: Optional[mp.Event] = None
        self._cmd_queue: Optional[mp.Queue] = None
        self._prev_combo_state = False

        self.get_logger().info(
            "SafePilot Teleop (Gesture Edition - Multi-Core) ready.\n"
            "   Press SELECT + B simultaneously to toggle Hand Gesture Control."
        )
        self._print_speed()

    # ==========================================================================
    #  Gesture Lifecycle Management
    # ==========================================================================
    def _start_gesture_mode(self):
        if self.gesture_mode:
            return

        self.get_logger().info("[GESTURE] Entering Hand Gesture Control mode ...")
        self._publish_status("GESTURE MODE ON")

        self.dc.is_holding = False
        if not self.dc.offboard_on:
            self._run_async(self.dc.cmd_enable_offboard())

        # Create IPC constructs
        self._cmd_queue = mp.Queue(maxsize=30)
        self._gesture_stop = mp.Event()

        # Dedicate cores for vision: choose high-index cores if multi-core system
        ncpus = os.cpu_count() or 4
        if ncpus >= 8:
            # Dedicate 4 performance threads to vision
            target_cores = list(range(ncpus - 4, ncpus))
        elif ncpus >= 4:
            target_cores = [ncpus - 2, ncpus - 1]
        else:
            target_cores = None

        self._gesture_proc = GestureVisionProcess(
            cmd_queue=self._cmd_queue,
            stop_event=self._gesture_stop,
            target_cores=target_cores,
        )
        self._gesture_proc.start()
        self.gesture_mode = True

        self.get_logger().info(
            f"[GESTURE] Vision worker process started (PID {self._gesture_proc.pid})."
        )

    def _stop_gesture_mode(self):
        if not self.gesture_mode:
            return

        self.get_logger().info("[GESTURE] Exiting Hand Gesture Control mode ...")

        # Signal vision process to exit
        if self._gesture_stop is not None:
            self._gesture_stop.set()

        # Join gracefully with fast timeout, falling back to termination
        if self._gesture_proc is not None:
            if self._gesture_proc.is_alive():
                self._gesture_proc.join(timeout=0.6)
            if self._gesture_proc.is_alive():
                self._gesture_proc.terminate()
                self._gesture_proc.join(timeout=0.4)

        # Drain queue to prevent Python multiprocessing queue hang
        if self._cmd_queue is not None:
            try:
                while not self._cmd_queue.empty():
                    self._cmd_queue.get_nowait()
                self._cmd_queue.close()
                self._cmd_queue.join_thread()
            except Exception:
                pass

        self.gesture_mode   = False
        self._gesture_proc  = None
        self._gesture_stop  = None
        self._cmd_queue     = None

        # Restore hover hold on drone
        self.dc.is_holding = True
        self.dc.set_velocity(0.0, 0.0, 0.0, 0.0)
        self.get_logger().info("[GESTURE] Joystick control restored. Drone in HOLD.")
        self._publish_status("GESTURE MODE OFF -- joystick restored")

    def _gesture_poll_callback(self):
        """
        Executed at 50 Hz by the ROS 2 executor thread.
        Pulls queued commands from the vision process safely.
        """
        if not self.gesture_mode or self._cmd_queue is None:
            return

        # Check for unexpected vision process termination
        if self._gesture_proc is not None and not self._gesture_proc.is_alive():
            self.get_logger().warn("[GESTURE] Vision process exited unexpectedly!")
            self._stop_gesture_mode()
            return

        # Drain pending queue items
        while True:
            try:
                msg = self._cmd_queue.get_nowait()
            except Exception:
                break

            cmd_type = msg[0]
            if cmd_type == 'exit':
                self.get_logger().info("[GESTURE] Vision window closed or requested exit. Restoring joystick.")
                self._stop_gesture_mode()
                return

            elif cmd_type == 'vel':
                if len(msg) >= 5:
                    _, vx, vy, vz, yaw = msg[:5]
                else:
                    _, vx, vy, vz = msg[:4]
                    yaw = 0.0
                if self.dc.is_holding:
                    self.dc.is_holding = False
                if self.dc.offboard_on:
                    self.dc.set_velocity(vx, vy, vz, yaw)

            elif cmd_type == 'hold':
                self.dc.is_holding = True
                self.dc.set_velocity(0.0, 0.0, 0.0, 0.0)
                if not self.dc.offboard_on:
                    self._run_async(self.dc.cmd_enable_offboard())
                self.get_logger().warn("[GESTURE] Hand lost -> drone in HOLD.")
                self._publish_status("GESTURE HOLD -- no hand detected")

            elif cmd_type in ('calibrating', 'recalibrating'):
                self.dc.is_holding = True
                self.dc.set_velocity(0.0, 0.0, 0.0, 0.0)
                if not self.dc.offboard_on:
                    self._run_async(self.dc.cmd_enable_offboard())
                if cmd_type == 'recalibrating':
                    self.get_logger().info("[GESTURE] Re-calibrating neutral hand pose...")
                    self._publish_status("GESTURE RE-CALIBRATING")

    # ==========================================================================
    #  Joystick Helpers & Callbacks
    # ==========================================================================
    def _btn(self, msg: Joy, idx: int) -> int:
        return int(msg.buttons[idx]) if idx < len(msg.buttons) else 0

    def _axis(self, msg: Joy, idx: int) -> float:
        return float(msg.axes[idx]) if idx < len(msg.axes) else 0.0

    def _rising_edge(self, name: str, cur: int) -> bool:
        prev = self._prev.get(name, 0)
        self._prev[name] = cur
        return cur == 1 and prev == 0

    def _print_speed(self):
        self.get_logger().info(
            "\n"
            "===========================\n"
            f" XY  speed : {self.vel_scale_xy:.2f} m/s\n"
            f" Z   speed : {self.vel_scale_z:.2f} m/s\n"
            f" Yaw speed : {self.yaw_scale:.1f} deg/s\n"
            "==========================="
        )

    def _run_async(self, coro):
        if self.dc.loop is None:
            self.get_logger().warn("Drone asyncio loop not initialized yet!")
            try:
                coro.close()
            except Exception:
                pass
            return
        asyncio.run_coroutine_threadsafe(coro, self.dc.loop)

    def joy_callback(self, msg: Joy):
        # 1. Gesture toggle combination: SELECT (btn 10) + B (btn 1)
        btn_select   = self._btn(msg, self.BTN_TELEMETRY)
        btn_b        = self._btn(msg, self.BTN_HOLD)
        combo_active = (btn_select == 1 and btn_b == 1)

        if combo_active and not self._prev_combo_state:
            if self.gesture_mode:
                self._stop_gesture_mode()
            else:
                self._start_gesture_mode()
        self._prev_combo_state = combo_active

        # 2. While in gesture mode, lock out joystick axes and single buttons
        if self.gesture_mode:
            self._prev['arm_takeoff']   = self._btn(msg, self.BTN_ARM_TAKEOFF)
            self._prev['land']          = self._btn(msg, self.BTN_LAND)
            self._prev['rtl']           = self._btn(msg, self.BTN_RTL)
            self._prev['hold']          = btn_b
            self._prev['start_mission'] = self._btn(msg, self.BTN_START_MISSION)
            self._prev['pause_mission'] = self._btn(msg, self.BTN_PAUSE_MISSION)
            self._prev['offboard_on']   = self._btn(msg, self.BTN_OFFBOARD_ON)
            self._prev['offboard_off']  = self._btn(msg, self.BTN_OFFBOARD_OFF)
            self._prev['telem']         = btn_select
            self._prev['kill']          = self._btn(msg, self.BTN_KILL)
            self._prev['geofence_on']   = self._btn(msg, self.BTN_GEOFENCE_ON)
            self._prev['geofence_off']  = self._btn(msg, self.BTN_GEOFENCE_OFF)
            self._prev['dpad_v']        = int(self._axis(msg, self.AXIS_DPAD_V))
            self._prev['dpad_h']        = int(self._axis(msg, self.AXIS_DPAD_H))
            return

        # 3. D-Pad dynamic speed scale adjustments
        dpad_v = int(self._axis(msg, self.AXIS_DPAD_V))
        if dpad_v != self._prev.get('dpad_v', 0):
            if dpad_v == 1:
                self.vel_scale_xy = min(self.VEL_XY_MAX, self.vel_scale_xy * 1.15)
                self.vel_scale_z  = min(self.VEL_Z_MAX,  self.vel_scale_z  * 1.15)
                self._print_speed()
            elif dpad_v == -1:
                self.vel_scale_xy = max(self.VEL_XY_MIN, self.vel_scale_xy * 0.85)
                self.vel_scale_z  = max(self.VEL_Z_MIN,  self.vel_scale_z  * 0.85)
                self._print_speed()
        self._prev['dpad_v'] = dpad_v

        dpad_h = int(self._axis(msg, self.AXIS_DPAD_H))
        if dpad_h != self._prev.get('dpad_h', 0):
            if dpad_h == -1:
                self.yaw_scale = min(self.YAW_MAX, self.yaw_scale * 1.15)
                self._print_speed()
            elif dpad_h == 1:
                self.yaw_scale = max(self.YAW_MIN, self.yaw_scale * 0.85)
                self._print_speed()
        self._prev['dpad_h'] = dpad_h

        # 4. Analog stick inputs -> velocity body frame
        raw_fwd    = self._axis(msg, self.AXIS_FWD)
        raw_strafe = self._axis(msg, self.AXIS_STRAFE)
        raw_throt  = self._axis(msg, self.AXIS_THROT)
        raw_yaw    = self._axis(msg, self.AXIS_YAW)

        def apply_deadband(val: float) -> float:
            return val if abs(val) > DEADBAND else 0.0

        raw_fwd    = apply_deadband(raw_fwd)
        raw_strafe = apply_deadband(raw_strafe)
        raw_throt  = apply_deadband(raw_throt)
        raw_yaw    = apply_deadband(raw_yaw)

        vx  =  raw_fwd    * self.vel_scale_xy
        vy  = -raw_strafe * self.vel_scale_xy
        vz  = -raw_throt  * self.vel_scale_z
        yaw = -raw_yaw    * self.yaw_scale

        if self.dc.offboard_on and not self.dc.is_holding:
            self.dc.set_velocity(vx, vy, vz, yaw)

        # 5. Rising-edge action buttons
        if self._rising_edge('arm_takeoff', self._btn(msg, self.BTN_ARM_TAKEOFF)):
            self.get_logger().info("[Y] ARM + TAKEOFF")
            self._run_async(self.dc.cmd_arm_takeoff(TAKEOFF_ALT))
            self._publish_status("ARM+TAKEOFF requested")

        if self._rising_edge('land', self._btn(msg, self.BTN_LAND)):
            self.get_logger().info("[A] LAND")
            self._run_async(self.dc.cmd_land())
            self._publish_status("LAND requested")

        if self._rising_edge('rtl', self._btn(msg, self.BTN_RTL)):
            self.get_logger().info("[X] RTL")
            self._run_async(self.dc.cmd_rtl())
            self._publish_status("RTL requested")

        if not combo_active and self._rising_edge('hold', btn_b):
            self.get_logger().info("[B] HOLD")
            self._run_async(self.dc.cmd_hold())
            self._publish_status("HOLD requested")
        else:
            self._prev['hold'] = btn_b

        if self._rising_edge('start_mission', self._btn(msg, self.BTN_START_MISSION)):
            if self.dc.mission_paused:
                self.get_logger().info("[L1] RESUME MISSION")
                self._run_async(self.dc.cmd_resume_mission())
                self._publish_status("MISSION RESUME requested")
            else:
                self.get_logger().info("[L1] START MISSION")
                self._run_async(self.dc.cmd_start_mission())
                self._publish_status("MISSION START requested")

        if self._rising_edge('pause_mission', self._btn(msg, self.BTN_PAUSE_MISSION)):
            self.get_logger().info("[R1] PAUSE MISSION")
            self._run_async(self.dc.cmd_pause_mission())
            self._publish_status("MISSION PAUSE requested")

        if self._rising_edge('offboard_on', self._btn(msg, self.BTN_OFFBOARD_ON)):
            self.get_logger().info("[L2] OFFBOARD ON")
            self._run_async(self.dc.cmd_enable_offboard())
            self._publish_status("OFFBOARD ON")

        if self._rising_edge('offboard_off', self._btn(msg, self.BTN_OFFBOARD_OFF)):
            self.get_logger().info("[R2] OFFBOARD OFF")
            self._run_async(self.dc.cmd_disable_offboard())
            self._publish_status("OFFBOARD OFF")

        if not combo_active and self._rising_edge('telem', btn_select):
            self.get_logger().info(f"[TELEMETRY] {self.dc.get_telemetry_str()}")
        else:
            self._prev['telem'] = btn_select

        if self._rising_edge('kill', self._btn(msg, self.BTN_KILL)):
            self.get_logger().fatal("[START] EMERGENCY KILL ISSUED!")
            self._run_async(self.dc.cmd_kill())
            self._publish_status("EMERGENCY KILL")

        if self._rising_edge('geofence_on', self._btn(msg, self.BTN_GEOFENCE_ON)):
            self.get_logger().info("[BTN 13] Enable Geofence")
            self._run_async(self.dc.cmd_enable_geofence())
            self._publish_status("GEOFENCE ENABLED")

        if self._rising_edge('geofence_off', self._btn(msg, self.BTN_GEOFENCE_OFF)):
            self.get_logger().info("[BTN 14] Disable Geofence")
            self._run_async(self.dc.cmd_disable_geofence())
            self._publish_status("GEOFENCE DISABLED")

    def _publish_status(self, msg_str: str):
        msg = String()
        msg.data = msg_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"[drone_status] -> '{msg_str}'")

    def destroy_node(self):
        if self.gesture_mode:
            self._stop_gesture_mode()
        super().destroy_node()


# ===============================================================================
#  Entry Point
# ===============================================================================
def main(args=None):
    # Use spawn start method for robust clean multiprocessing
    try:
        mp.set_start_method('spawn', force=True)
    except RuntimeError:
        pass

    dc = DroneController()

    def start_mavsdk_loop():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        dc.loop = loop
        loop.run_until_complete(dc.run())

    mavsdk_thread = threading.Thread(target=start_mavsdk_loop, daemon=True, name="MAVSDKAsyncLoop")
    mavsdk_thread.start()

    time.sleep(0.5)

    rclpy.init(args=args)
    node = DroneJoyTeleop(dc)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down SafePilot Gesture Teleop node ...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
