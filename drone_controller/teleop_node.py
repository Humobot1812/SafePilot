#!/usr/bin/env python3
"""
Semi-Autonomous Drone Joystick Teleop Node
==========================================
Bridges a PS2/Xbox joystick (via /joy) to a MAVSDK-connected drone.

Controller Layout
-----------------
RIGHT STICK (axis 3 = up/down)   → Forward / Backward  (body X velocity)
RIGHT STICK (axis 2 = left/right) → Strafe  Left / Right (body Y velocity)
LEFT STICK  (axis 1 = up/down)   → Throttle Up / Down   (body Z velocity, negative = up)
LEFT STICK  (axis 0 = left/right) → Yaw Left / Right    (yaw-rate deg/s)

D-PAD Up / Down                  → Increase / Decrease linear speed scale
D-PAD Left / Right               → Increase / Decrease yaw-rate scale

Face Buttons (rising-edge):
  Y (btn 4)  → ARM + TAKEOFF  (starts offboard after climb)
  A (btn 0)  → LAND           (disarm on ground)
  X (btn 3)  → RTL            (Return-to-Launch)
  B (btn 1)  → HOLD / LOITER  (stop and hover in place)

Shoulder Buttons (rising-edge):
  L1 (btn 6)  → START MISSION  (switches from offboard to mission mode)
  R1 (btn 7)  → PAUSE MISSION  (pauses current mission, hovers)
  L2 (btn 8)  → Toggle OFFBOARD manual control ON
  R2 (btn 9)  → Toggle OFFBOARD manual control OFF (sends zero velocity)

Select / Start:
  SELECT (btn 10) → Print current telemetry (lat/lon/alt) to log
  START  (btn 11) → Emergency KILL (disarm immediately — use with caution!)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import asyncio
import threading
import time
import math

# ─── MAVSDK imports ────────────────────────────────────────────────────────────
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from mavsdk.action import ActionError
from mavsdk.geofence import Point, Polygon, FenceType, GeofenceData
from mavsdk.mission import MissionItem, MissionPlan


# ═══════════════════════════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════════════════════════
DRONE_ADDRESS   = "udpin://0.0.0.0:14550"  # MAVLink UDP endpoint
TAKEOFF_ALT     = 5.0                       # metres (default takeoff altitude)
MAX_VEL_XY      = 10.0                       # m/s  (max horizontal speed)
MAX_VEL_Z       = 10.0                       # m/s  (max vertical speed)
MAX_YAW_RATE    = 60.0                       # deg/s (max yaw rate)
OFFBOARD_HZ     = 20                         # frequency of offboard set-point loop
DEADBAND        = 0.05                       # joystick axis dead-band

# ── Geofence Configuration (Embedded) ─────────────────────────────────────────
FENCE_ALT_MAX   = 50.0    # Max altitude ceiling (meters)
FENCE_ACTION    = 1       # 1 = Return To Launch (RTL) on breach, 2 = Land

# Inclusion polygon coordinates: (latitude, longitude)
GEOFENCE_POLYGON = [
    # (23.176860779846585, 80.0221183906635),
    # (23.176497691583677, 80.02220781636967),
    # (23.17648399012051, 80.02179546450226),
    # (23.17682652627896, 80.02172591117522),
    (-35.36324697,149.16615638),
    (-35.36340159,149.16514499),
    (-35.36253012,149.16494645),
    (-35.36237550,149.16595783),
    (-35.36324697,149.16615638)
]

# ── Auto Mission Configuration (Empty Placeholder Data) ───────────────────────
# Fill AUTO_WAYPOINTS with (latitude, longitude) tuples to enable auto mission
# AUTO_WAYPOINTS  = []      # e.g., [(23.1768, 80.0221), (23.1765, 80.0222)]
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

AUTO_ALTITUDE   = 15.0    # Autonomous mission altitude (meters)
AUTO_SPEED      = 5.0     # Autonomous mission flight speed (m/s)


# ═══════════════════════════════════════════════════════════════════════════════
#  Drone Flight Controller  (runs in its own asyncio event loop / thread)
# ═══════════════════════════════════════════════════════════════════════════════
class DroneController:
    """
    Wraps a MAVSDK System and exposes simple coroutines that the ROS2 node
    can call safely via asyncio.run_coroutine_threadsafe().
    """

    def __init__(self):
        self.drone                = System()
        self.connected            = False
        self.armed                = False
        self.in_air               = False
        self.offboard_on          = False
        self.mission_active       = False
        self.mission_paused       = False   # True after pause_mission; cleared on resume or fresh start
        self.mission_uploaded     = False   # True once we have successfully uploaded the plan
        self.is_holding           = False

        # Tracks the waypoint index at the moment of the last pause so we can
        # resume from exactly that point on every pause/resume cycle.
        self._current_mission_item = 0      # live index, updated by the progress stream
        self._paused_mission_item  = 0      # snapshot taken on pause

        # Desired velocity set-point (updated by joystick, read by offboard loop)
        self._vx  = 0.0   # forward  (m/s)
        self._vy  = 0.0   # rightward (m/s)
        self._vz  = 0.0   # downward  (m/s, positive = descend for MAVSDK)
        self._yaw = 0.0   # yaw rate  (deg/s, positive = clockwise)

        self._lock = threading.Lock()

        # Telemetry cache
        self.telem_lat = 0.0
        self.telem_lon = 0.0
        self.telem_alt = 0.0
        self.telem_heading = 0.0

        # asyncio event loop (set when the thread starts)
        self.loop: asyncio.AbstractEventLoop = None

    # ── velocity set-point (thread-safe) ──────────────────────────────────────
    def set_velocity(self, vx: float, vy: float, vz: float, yaw: float):
        with self._lock:
            self._vx  = vx
            self._vy  = vy
            self._vz  = vz
            self._yaw = yaw

    def get_velocity(self):
        with self._lock:
            return self._vx, self._vy, self._vz, self._yaw

    # ── asyncio entry point ────────────────────────────────────────────────────
    async def run(self):
        """Connect to drone, stream telemetry, run offboard loop forever."""
        print("[DroneController] Connecting to drone …")
        await self.drone.connect(system_address=DRONE_ADDRESS)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.connected = True
                print("[DroneController] ✅ Connected to drone!")
                break

        # Stream telemetry in background tasks
        asyncio.ensure_future(self._stream_position())
        asyncio.ensure_future(self._stream_armed())
        asyncio.ensure_future(self._stream_in_air())
        asyncio.ensure_future(self._stream_geofence_status())
        asyncio.ensure_future(self._stream_mission_progress())

        # Offboard set-point loop (runs even when offboard is not active;
        # the actual mode switch is gated by self.offboard_on)
        asyncio.ensure_future(self._offboard_loop())

        print("[DroneController] Background tasks started. Waiting for commands …")

        # Keep the coroutine alive
        while True:
            await asyncio.sleep(1.0)

    # ── telemetry streams ──────────────────────────────────────────────────────
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
        """Monitor geofence: if breached, stop offboard loop so FC can execute RTL action!"""
        try:
            async for result in self.drone.geofence.geofence_result():
                if hasattr(result, 'is_breached') and result.is_breached:
                    print("[DroneController] 🚨 GEOFENCE BREACH DETECTED! Disabling offboard stream for RTL.")
                    self.offboard_on = False
        except Exception:
            pass

    async def _stream_mission_progress(self):
        """Continuously track the current mission waypoint index.

        This is the key to reliable multi-cycle pause/resume: we always know
        the exact waypoint the drone is flying toward, so we can explicitly
        tell ArduPilot to resume from that index instead of relying on
        start_mission()'s default behaviour (which restarts from WP 0).

        Also detects mission completion and automatically triggers RTL.
        """
        try:
            async for progress in self.drone.mission.mission_progress():
                self._current_mission_item = progress.current

                # Auto-RTL when mission finishes
                # MAVSDK reports current == total when all items are done
                if (self.mission_active
                        and progress.total > 0
                        and progress.current >= progress.total):
                    print("[DroneController] ✅ Mission complete! Triggering RTL …")
                    self.mission_active = False
                    self.mission_paused = False
                    asyncio.ensure_future(self.cmd_rtl())
        except Exception:
            pass

    # ── offboard set-point loop ────────────────────────────────────────────────
    async def _offboard_loop(self):
        """
        Continuously sends velocity body set-points when offboard is active.
        Sending at OFFBOARD_HZ keeps the flight controller happy.
        """
        dt = 1.0 / OFFBOARD_HZ
        while True:
            if self.offboard_on:
                vx, vy, vz, yaw = self.get_velocity()
                try:
                    await self.drone.offboard.set_velocity_body(
                        VelocityBodyYawspeed(vx, vy, vz, yaw)
                    )
                except OffboardError as e:
                    print(f"[DroneController] Offboard set error: {e}")
            await asyncio.sleep(dt)

    # ── action coroutines (called from ROS2 thread via run_coroutine_threadsafe) ─
    async def cmd_arm_takeoff(self, altitude: float = TAKEOFF_ALT):
        if not self.connected:
            print("[DroneController] ⚠ Not connected — cannot arm/takeoff.")
            return
        try:
            print("[DroneController] Arming …")
            await self.drone.action.arm()

            print(f"[DroneController] Taking off to {altitude} m …")
            await self.drone.action.set_takeoff_altitude(altitude)
            await self.drone.action.takeoff()

            # Wait until we reach 95 % of target altitude
            async for pos in self.drone.telemetry.position():
                print(f"[DroneController] Altitude: {pos.relative_altitude_m:.1f} m")
                if pos.relative_altitude_m >= altitude * 0.95:
                    print("[DroneController] ✅ Reached target altitude.")
                    break

            # Immediately enter offboard so the joystick has control
            await self._start_offboard()

        except ActionError as e:
            print(f"[DroneController] Arm/Takeoff error: {e}")

    async def cmd_land(self):
        try:
            print("[DroneController] Landing …")
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.action.land()
        except ActionError as e:
            print(f"[DroneController] Land error: {e}")

    async def cmd_rtl(self):
        try:
            print("[DroneController] RTL …")
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.action.return_to_launch()
        except ActionError as e:
            print(f"[DroneController] RTL error: {e}")

    async def cmd_hold(self):
        """Command zero velocity and stay in offboard (hover in place), ignoring stick inputs."""
        print("[DroneController] HOLD — zeroing velocity & locking sticks.")
        self.is_holding = True
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
        if not self.offboard_on:
            await self._start_offboard()

    async def cmd_start_mission(self):
        """
        Upload mission plan and start from the beginning (fresh start).

        - If the drone is NOT in the air: arms, climbs to AUTO_ALTITUDE,
          then uploads and starts the mission automatically.
        - If the drone IS already in the air: uploads and starts immediately.
        - After the last waypoint: RTL is triggered automatically via the
          _stream_mission_progress() watcher.
        """
        try:
            if not AUTO_WAYPOINTS:
                print("[DroneController] ⚠ AUTO_WAYPOINTS is empty! Add waypoints to teleop_node.py")
                return

            # ── 1. Takeoff if not already in air ───────────────────────────────────
            if not self.in_air:
                if not self.connected:
                    print("[DroneController] ⚠ Not connected — cannot start mission.")
                    return

                print("[DroneController] Drone is on the ground — arming and taking off before mission ...")

                # Stop offboard if it was active
                if self.offboard_on:
                    await self._stop_offboard()

                print("[DroneController] Arming …")
                await self.drone.action.arm()

                print(f"[DroneController] Taking off to {AUTO_ALTITUDE} m …")
                await self.drone.action.set_takeoff_altitude(AUTO_ALTITUDE)
                await self.drone.action.takeoff()

                # Wait until 95% of mission altitude is reached
                async for pos in self.drone.telemetry.position():
                    print(f"[DroneController] Altitude: {pos.relative_altitude_m:.1f} m  (target {AUTO_ALTITUDE} m)")
                    if pos.relative_altitude_m >= AUTO_ALTITUDE * 0.95:
                        print("[DroneController] ✅ Reached mission altitude.")
                        break
            else:
                print("[DroneController] Drone already in air — starting mission directly.")

                # Exit offboard so we can switch to mission mode
                if self.offboard_on:
                    await self._stop_offboard()

            # ── 2. Upload mission plan ───────────────────────────────────────
            print(f"[DroneController] Uploading mission ({len(AUTO_WAYPOINTS)} waypoints) …")
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
                    vehicle_action=MissionItem.VehicleAction.NONE
                ))
            await self.drone.mission.upload_mission(MissionPlan(mission_items))
            # RTL is also handled by the mission_progress watcher, but set this
            # as a FC-side safety net in case the watcher misses the transition.
            await self.drone.mission.set_return_to_launch_after_mission(True)
            self.mission_uploaded = True

            # ── 3. Start mission ──────────────────────────────────────────
            print("[DroneController] 🚀 Starting mission …")
            await self.drone.mission.start_mission()
            self.mission_active = True
            self.mission_paused = False
        except Exception as e:
            print(f"[DroneController] Start mission error: {e}")

    async def cmd_resume_mission(self):
        """
        Resume a paused mission from the EXACT waypoint that was active when
        R1 was pressed — works correctly for any number of pause/resume cycles.

        Root cause of the "only works once" bug:
          ArduPilot's start_mission() (MAV_CMD_MISSION_START param1=0) always
          resets to waypoint 0, not the paused index. The fix is to call
          set_current_mission_item(saved_index) first, which explicitly tells
          the FC which item to jump to before starting.
        """
        try:
            if not self.mission_uploaded:
                print("[DroneController] No mission uploaded yet — performing fresh start instead.")
                await self.cmd_start_mission()
                return

            resume_idx = self._paused_mission_item
            print(f"[DroneController] Resuming mission from waypoint {resume_idx} …")

            if self.offboard_on:
                await self._stop_offboard()

            # Explicitly set the waypoint index so ArduPilot doesn't reset to 0
            await self.drone.mission.set_current_mission_item(resume_idx)
            await self.drone.mission.start_mission()

            self.mission_active = True
            self.mission_paused = False
        except Exception as e:
            print(f"[DroneController] Resume mission error: {e}")

    async def cmd_pause_mission(self):
        try:
            # Snapshot the current waypoint index BEFORE pausing so resume
            # can return to this exact point on every pause/resume cycle.
            self._paused_mission_item = self._current_mission_item
            print(f"[DroneController] Pausing mission at waypoint {self._paused_mission_item} …")

            await self.drone.mission.pause_mission()
            self.mission_active = False
            self.mission_paused = True

            # Enter offboard so the pilot can re-position manually
            await self._start_offboard()
        except Exception as e:
            print(f"[DroneController] Pause mission error: {e}")

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
        """Emergency kill — disarms immediately regardless of state."""
        print("[DroneController] ⚠⚠⚠  EMERGENCY KILL — DISARMING! ⚠⚠⚠")
        try:
            if self.offboard_on:
                await self._stop_offboard()
            await self.drone.action.kill()
        except ActionError as e:
            print(f"[DroneController] Kill error: {e}")

    async def cmd_enable_geofence(self):
        """Upload embedded polygon geofence and enable FC fence protection."""
        print("[DroneController] Uploading and enabling Geofence ...")
        try:
            await self.drone.geofence.clear_geofence()
            # FENCE_TYPE: 1=Alt, 2=Circle, 4=Polygon, 7=All Enabled
            await self.drone.param.set_param_int("FENCE_TYPE", 7)
            await self.drone.param.set_param_int("FENCE_ENABLE", 1)
            await self.drone.param.set_param_int("FENCE_ACTION", FENCE_ACTION)
            await self.drone.param.set_param_float("FENCE_ALT_MAX", float(FENCE_ALT_MAX))

            if GEOFENCE_POLYGON:
                points = [Point(lat, lon) for lat, lon in GEOFENCE_POLYGON]
                polygon = Polygon(points, FenceType.INCLUSION)
                geofence_data = GeofenceData([polygon], [])
                await self.drone.geofence.upload_geofence(geofence_data)
                print(f"[DroneController] ✅ Geofence uploaded ({len(points)} points) & ENABLED (Max Alt: {FENCE_ALT_MAX}m)")
            else:
                print(f"[DroneController] ✅ Geofence parameters ENABLED (Max Alt: {FENCE_ALT_MAX}m)")
        except Exception as e:
            print(f"[DroneController] Enable Geofence error: {e}")

    async def cmd_disable_geofence(self):
        """Disable Geofence and clear fence data."""
        print("[DroneController] Disabling Geofence ...")
        try:
            await self.drone.param.set_param_int("FENCE_ENABLE", 0)
            await self.drone.geofence.clear_geofence()
            print("[DroneController] 🚫 Geofence DISABLED & cleared")
        except Exception as e:
            print(f"[DroneController] Disable Geofence error: {e}")

    async def _start_offboard(self):
        """Zero velocity, then enable offboard mode."""
        self.set_velocity(0.0, 0.0, 0.0, 0.0)
        try:
            # Must send at least one set-point before enabling offboard
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await self.drone.offboard.start()
            self.offboard_on = True
            print("[DroneController] ✅ Offboard mode ON — joystick in control.")
        except OffboardError as e:
            print(f"[DroneController] Offboard start error: {e}")

    async def _stop_offboard(self):
        try:
            await self.drone.offboard.stop()
            self.offboard_on = False
            print("[DroneController] Offboard mode OFF.")
        except OffboardError as e:
            print(f"[DroneController] Offboard stop error: {e}")

    def get_telemetry_str(self):
        """Return formatted string of cached telemetry."""
        return (
            f"Lat={self.telem_lat:.7f} | "
            f"Lon={self.telem_lon:.7f} | "
            f"Alt={self.telem_alt:.2f}m | "
            f"Armed={self.armed} | InAir={self.in_air} | "
            f"Offboard={self.offboard_on} | Holding={self.is_holding}"
        )


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 Node
# ═══════════════════════════════════════════════════════════════════════════════
class DroneJoyTeleop(Node):
    """
    ROS2 node that reads /joy and translates it to MAVSDK drone commands.
    Joystick axes control 6-DOF velocity (offboard mode).
    Buttons trigger discrete actions (arm, land, RTL, mission, etc.).
    """

    def __init__(self, drone_ctrl: DroneController):
        super().__init__('drone_joy_teleop')
        self.dc = drone_ctrl

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # ── Status publisher ───────────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/drone_status', 10)

        # ── Axis mapping ───────────────────────────────────────────────────────
        #  Adjust if your controller reports different indices
        self.AXIS_FWD   = 3   # Right stick vertical   (+ = fwd)
        self.AXIS_STRAFE= 2   # Right stick horizontal (+ = left → mapped to -Y)
        self.AXIS_THROT = 1   # Left  stick vertical   (+ = up   → mapped to -Z)
        self.AXIS_YAW   = 0   # Left  stick horizontal (+ = left → mapped to -yaw)

        self.AXIS_DPAD_V = 7  # D-pad vertical   (some controllers expose as axis)
        self.AXIS_DPAD_H = 6  # D-pad horizontal6

        # ── Button mapping ─────────────────────────────────────────────────────
        self.BTN_ARM_TAKEOFF    = 4   # Y
        self.BTN_LAND           = 0   # A
        self.BTN_RTL            = 3   # X
        self.BTN_HOLD           = 1   # B
        self.BTN_START_MISSION  = 7   # R1
        self.BTN_PAUSE_MISSION  = 9   # R2
        self.BTN_OFFBOARD_ON    = 6   # L1
        self.BTN_OFFBOARD_OFF   = 8   # L2
        self.BTN_TELEMETRY      = 10  # SELECT
        self.BTN_KILL           = 11  # START  (emergency!)
        self.BTN_GEOFENCE_ON    = 13  # Button 13 → Enable Geofence
        self.BTN_GEOFENCE_OFF   = 14  # Button 14 → Disable Geofence

        # ── Speed scales (adjustable via D-pad) ───────────────────────────────
        self.vel_scale_xy  = 2.0    # m/s per full stick deflection
        self.vel_scale_z   = 1.0    # m/s per full stick deflection
        self.yaw_scale     = 30.0   # deg/s per full stick deflection

        self.VEL_XY_MIN  = 0.2;  self.VEL_XY_MAX  = MAX_VEL_XY
        self.VEL_Z_MIN   = 0.2;  self.VEL_Z_MAX   = MAX_VEL_Z
        self.YAW_MIN     = 5.0;  self.YAW_MAX     = MAX_YAW_RATE

        # ── Debounce: previous button states ──────────────────────────────────
        self._prev = {}

        self.get_logger().info("🚁 Drone Joy Teleop node started.")
        self._print_speed()

    # ── Helper: safe button read ───────────────────────────────────────────────
    def _btn(self, msg: Joy, idx: int) -> int:
        return int(msg.buttons[idx]) if idx < len(msg.buttons) else 0

    def _axis(self, msg: Joy, idx: int) -> float:
        return float(msg.axes[idx]) if idx < len(msg.axes) else 0.0

    def _rising_edge(self, name: str, cur: int) -> bool:
        prev = self._prev.get(name, 0)
        self._prev[name] = cur
        return cur == 1 and prev == 0

    # ── Helper: print current speed ───────────────────────────────────────────
    def _print_speed(self):
        self.get_logger().info(
            "\n"
            "===========================\n"
            f" XY  speed : {self.vel_scale_xy:.2f} m/s\n"
            f" Z   speed : {self.vel_scale_z:.2f} m/s\n"
            f" Yaw speed : {self.yaw_scale:.1f} deg/s\n"
            "==========================="
        )

    # ── Helper: schedule async commands safely ────────────────────────────────
    def _run_async(self, coro):
        """Submit a coroutine to the drone's event loop from the ROS2 thread."""
        if self.dc.loop is None:
            self.get_logger().warn("Drone event loop not ready yet!")
            return
        asyncio.run_coroutine_threadsafe(coro, self.dc.loop)

    # ── Main joystick callback ─────────────────────────────────────────────────
    def joy_callback(self, msg: Joy):

        # ── 1. D-pad speed tuning ─────────────────────────────────────────────
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

        # ── 2. Velocity set-points from analog sticks ─────────────────────────
        raw_fwd    = self._axis(msg, self.AXIS_FWD)
        raw_strafe = self._axis(msg, self.AXIS_STRAFE)
        raw_throt  = self._axis(msg, self.AXIS_THROT)
        raw_yaw    = self._axis(msg, self.AXIS_YAW)

        # Apply dead-band
        def db(v):
            return v if abs(v) > DEADBAND else 0.0

        raw_fwd    = db(raw_fwd)
        raw_strafe = db(raw_strafe)
        raw_throt  = db(raw_throt)
        raw_yaw    = db(raw_yaw)

        # Map to MAVSDK VelocityBodyYawspeed convention:
        #   vx > 0 → forward | vy > 0 → right | vz > 0 → descend
        vx  =  raw_fwd    * self.vel_scale_xy   # forward / backward
        vy  = -raw_strafe * self.vel_scale_xy   # left stick right = strafe right
        vz  = -raw_throt  * self.vel_scale_z    # stick up = ascend (vz negative)
        yaw = -raw_yaw    * self.yaw_scale       # stick right = yaw right

        # Only update the drone if offboard is active and NOT currently locked in Hold mode
        if self.dc.offboard_on and not self.dc.is_holding:
            self.dc.set_velocity(vx, vy, vz, yaw)

        # ── 3. Action buttons (rising-edge) ───────────────────────────────────

        # Y → ARM + TAKEOFF
        if self._rising_edge('arm_takeoff', self._btn(msg, self.BTN_ARM_TAKEOFF)):
            self.get_logger().info("[Y] ARM + TAKEOFF")
            self._run_async(self.dc.cmd_arm_takeoff(TAKEOFF_ALT))
            self._publish_status("ARM+TAKEOFF requested")

        # A → LAND
        if self._rising_edge('land', self._btn(msg, self.BTN_LAND)):
            self.get_logger().info("[A] LAND")
            self._run_async(self.dc.cmd_land())
            self._publish_status("LAND requested")

        # X → RTL
        if self._rising_edge('rtl', self._btn(msg, self.BTN_RTL)):
            self.get_logger().info("[X] RTL")
            self._run_async(self.dc.cmd_rtl())
            self._publish_status("RTL requested")

        # B → HOLD (hover)
        if self._rising_edge('hold', self._btn(msg, self.BTN_HOLD)):
            self.get_logger().info("[B] HOLD")
            self._run_async(self.dc.cmd_hold())
            self._publish_status("HOLD requested")

        # L1 → START or RESUME MISSION
        # If mission was paused, resume from the paused waypoint (no re-upload).
        # If no mission is active/paused, do a fresh upload + start.
        if self._rising_edge('start_mission', self._btn(msg, self.BTN_START_MISSION)):
            if self.dc.mission_paused:
                self.get_logger().info("[L1] RESUME MISSION (continuing from paused waypoint)")
                self._run_async(self.dc.cmd_resume_mission())
                self._publish_status("MISSION RESUME requested")
            else:
                self.get_logger().info("[L1] START MISSION (fresh upload)")
                self._run_async(self.dc.cmd_start_mission())
                self._publish_status("MISSION START requested")

        # R1 → PAUSE MISSION
        if self._rising_edge('pause_mission', self._btn(msg, self.BTN_PAUSE_MISSION)):
            self.get_logger().info("[R1] PAUSE MISSION")
            self._run_async(self.dc.cmd_pause_mission())
            self._publish_status("MISSION PAUSE requested")

        # L2 → Enable offboard (manual joystick control)
        if self._rising_edge('offboard_on', self._btn(msg, self.BTN_OFFBOARD_ON)):
            self.get_logger().info("[L2] OFFBOARD ON — joystick control active")
            self._run_async(self.dc.cmd_enable_offboard())
            self._publish_status("OFFBOARD ON")

        # R2 → Disable offboard (drone holds position via FC)
        if self._rising_edge('offboard_off', self._btn(msg, self.BTN_OFFBOARD_OFF)):
            self.get_logger().info("[R2] OFFBOARD OFF — handing back to FC")
            self._run_async(self.dc.cmd_disable_offboard())
            self._publish_status("OFFBOARD OFF")

        # SELECT → Print telemetry
        if self._rising_edge('telem', self._btn(msg, self.BTN_TELEMETRY)):
            self.get_logger().info(f"[TELEMETRY] {self.dc.get_telemetry_str()}")

        # START → EMERGENCY KILL
        if self._rising_edge('kill', self._btn(msg, self.BTN_KILL)):
            self.get_logger().fatal("[START] ⚠ EMERGENCY KILL ISSUED!")
            self._run_async(self.dc.cmd_kill())
            self._publish_status("EMERGENCY KILL")

        # Button 13 → ENABLE GEOFENCE
        if self._rising_edge('geofence_on', self._btn(msg, self.BTN_GEOFENCE_ON)):
            self.get_logger().info("[BTN 13] Enable Geofence")
            self._run_async(self.dc.cmd_enable_geofence())
            self._publish_status("GEOFENCE ENABLED")

        # Button 14 → DISABLE GEOFENCE
        if self._rising_edge('geofence_off', self._btn(msg, self.BTN_GEOFENCE_OFF)):
            self.get_logger().info("[BTN 14] Disable Geofence")
            self._run_async(self.dc.cmd_disable_geofence())
            self._publish_status("GEOFENCE DISABLED")

    # ── Status publisher helper ────────────────────────────────────────────────
    def _publish_status(self, msg_str: str):
        msg = String()
        msg.data = msg_str
        self.status_pub.publish(msg)
        self.get_logger().info(f"[drone_status] → '{msg_str}'")


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    # ── 1. Create drone controller ────────────────────────────────────────────
    dc = DroneController()

    # ── 2. Start MAVSDK event loop in a background thread ────────────────────
    def start_mavsdk_loop():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        dc.loop = loop
        loop.run_until_complete(dc.run())

    mavsdk_thread = threading.Thread(target=start_mavsdk_loop, daemon=True)
    mavsdk_thread.start()

    # Give MAVSDK a moment to set up the loop reference before ROS spins
    time.sleep(0.5)

    # ── 3. Spin ROS2 node ─────────────────────────────────────────────────────
    rclpy.init(args=args)
    node = DroneJoyTeleop(dc)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down drone teleop node …")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()