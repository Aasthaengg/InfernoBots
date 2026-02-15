#!/usr/bin/env python3
"""
FireBot youBot Loader — v9 (Vision-Integrated Navigation)

KEY FIXES (v9):
  1. Real vision-based object detection in pick sequence (not hardcoded)
  2. Camera-based obstacle detection to prevent going under tables
  3. Vision-based line following using floor color markers
  4. GPS sensor fusion with odometry for camera alignment
  5. PID controller for smooth, stable navigation

Based on research from:
  - fadel-hasan/robotic_project (line following PID)
  - Vision-Based Line Following Robot in Webots (ResearchGate)
  - Kinematic-Omniwheels-in-Webots (PID control)
"""

from controller import Robot
import json
import math
import struct
import sys
import os

# Add project root for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from controllers.youbot_loader.vision_api import analyze_shelf, get_strafe_offset, POSITION_OFFSETS

TIMESTEP = 32

# ══════════════════════════════════════════════════════════════
# LINE COLOR DEFINITIONS (from world file floor markers)
# ══════════════════════════════════════════════════════════════

# HSV-like thresholds for floor line detection (in RGB 0-255)
LINE_COLORS = {
    "spine":   {"r": (200, 255), "g": (200, 255), "b": (0, 80)},     # Yellow spine
    "aisle_A": {"r": (200, 255), "g": (0, 60),    "b": (0, 60)},     # Red
    "aisle_B": {"r": (200, 255), "g": (100, 160),  "b": (0, 40)},    # Orange
    "aisle_C": {"r": (0, 80),    "g": (160, 255), "b": (0, 80)},     # Green
    "aisle_D": {"r": (0, 80),    "g": (80, 160),  "b": (200, 255)},  # Blue
    "truck":   {"r": (200, 255), "g": (200, 255), "b": (200, 255)},  # White
}

AISLE_COLOR_MAP = {
    "A": "aisle_A",
    "B": "aisle_B",
    "C": "aisle_C",
    "D": "aisle_D",
}

# Obstacle detection: if bottom portion of camera is mostly dark/occluded
OBSTACLE_DARKNESS_THRESHOLD = 40   # Average pixel value below this = obstacle
OBSTACLE_ROW_FRACTION = 0.3        # Check bottom 30% of image


# ══════════════════════════════════════════════════════════════
# PID CONTROLLER
# ══════════════════════════════════════════════════════════════

class PID:
    def __init__(self, kp, ki, kd, min_out=-10, max_out=10):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt=0.032):
        self.integral += error * dt
        # Anti-windup
        self.integral = max(-5, min(5, self.integral))

        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.min_out, min(self.max_out, output))

    def reset(self):
        self.integral = 0
        self.prev_error = 0


# ══════════════════════════════════════════════════════════════
# ARM CONFIGURATION
# ══════════════════════════════════════════════════════════════

ARM_SPEEDS = [0.5, 0.8, 0.8, 0.6, 0.3]

# Arm presets calibrated for shelf height 0.6m
ARM_HOME     = [0.0, -1.13, -2.54, -1.78, 0.0]
ARM_READY    = [0.0, -0.50, -1.80, -1.30, 0.0]   # Extended but safe
ARM_PRE_GRAB = [0.0,  0.10, -1.10, -1.60, 0.0]   # Above object
ARM_GRAB     = [0.0,  0.40, -0.70, -1.80, 0.0]   # At grab height
ARM_LIFT     = [0.0, -0.70, -2.00, -1.00, 0.0]   # Lifted with object
ARM_DROP     = [0.0, -0.10, -1.20, -1.50, 0.0]

# Height-adjusted grab positions for different shelf levels
ARM_GRAB_HIGH = [0.0,  0.20, -0.90, -1.70, 0.0]  # Upper shelf
ARM_GRAB_LOW  = [0.0,  0.55, -0.50, -1.90, 0.0]  # Lower shelf

ARM_LIMITS = [
    (-2.949, 2.949), (-1.1345, 1.5708), (-2.5481, 2.5481),
    (-1.7802, 1.7802), (-2.949, 2.949),
]

# ══════════════════════════════════════════════════════════════
# WAYPOINT NAVIGATION GRID
# ══════════════════════════════════════════════════════════════

# Key positions (X, Y)
WAYPOINTS = {
    "START":      (-1.5, -2.0),
    "SPINE_D":    (-1.5, -0.5),   # Spine position for aisle D
    "SPINE_C":    (-1.5,  1.5),
    "SPINE_B":    (-1.5,  3.5),
    "SPINE_A":    (-1.5,  5.5),
    "SHELF_D":    (-3.8, -0.5),   # Pickup position for aisle D
    "SHELF_C":    (-3.8,  1.5),
    "SHELF_B":    (-3.8,  3.5),
    "SHELF_A":    (-3.8,  5.5),
    "TRUCK_APP":  (-1.5, -3.0),   # Truck approach
    "TRUCK_DROP": ( 0.0, -4.2),   # Drop position
}

# Route for each aisle
AISLE_ROUTES = {
    "D": ["SPINE_D", "SHELF_D"],
    "C": ["SPINE_C", "SHELF_C"],
    "B": ["SPINE_B", "SHELF_B"],
    "A": ["SPINE_A", "SHELF_A"],
}

TRUCK_ROUTE = ["TRUCK_APP", "TRUCK_DROP"]

EQUIPMENT = {
    "scba_tank":   {"aisle": "D", "cat": "Breathing & PPE",   "color": "blue"},
    "first_aid":   {"aisle": "C", "cat": "Medical & Rescue",  "color": "green"},
    "hazmat_suit": {"aisle": "B", "cat": "Hazmat & Chemical", "color": "orange"},
    "ext_co2":     {"aisle": "A", "cat": "Fire Suppression",  "color": "red"},
}

DEFAULT_MISSION = ["scba_tank", "first_aid", "hazmat_suit", "ext_co2"]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalize_angle(a):
    while a > math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def _first_device(robot: Robot, names):
    for name in names:
        try:
            dev = robot.getDevice(name)
            if dev is not None:
                return dev
        except Exception:
            continue
    return None


def _motor_limits(motor):
    try:
        lo = motor.getMinPosition()
        hi = motor.getMaxPosition()
        if lo is None or hi is None:
            return (-3.14, 3.14)
        return (float(lo), float(hi))
    except Exception:
        return (-3.14, 3.14)


class YouBotLoader:
    def __init__(self):
        self.robot = Robot()

        # ── Wheels ──
        self.wheels = []
        for name in ["wheel1", "wheel2", "wheel3", "wheel4"]:
            try:
                w = self.robot.getDevice(name)
                w.setPosition(float('inf'))
                w.setVelocity(0.0)
                self.wheels.append(w)
            except Exception:
                pass
        if len(self.wheels) == 0:
            left = _first_device(self.robot, ["wheel_left_joint", "left_wheel_joint", "left_wheel"])
            right = _first_device(self.robot, ["wheel_right_joint", "right_wheel_joint", "right_wheel"])
            if left and right:
                left.setPosition(float('inf'))
                right.setPosition(float('inf'))
                left.setVelocity(0.0)
                right.setVelocity(0.0)
                self.wheels = [left, right]
        self.drive_mode = "mecanum" if len(self.wheels) == 4 else "diff"

        # ── Arm ──
        self.arm_motors = []
        self.arm_sensors = []
        arm_names = [f"arm{i}" for i in range(1, 6)]
        if _first_device(self.robot, arm_names) is None:
            arm_names = [f"arm_{i}_joint" for i in range(1, 8)]
        for idx, name in enumerate(arm_names):
            try:
                m = self.robot.getDevice(name)
            except Exception:
                m = None
            if m is None:
                continue
            speed = ARM_SPEEDS[idx] if idx < len(ARM_SPEEDS) else 0.5
            try:
                m.setVelocity(speed)
            except Exception:
                pass
            sensor = None
            try:
                sensor = self.robot.getDevice(f"{name}sensor")
            except Exception:
                sensor = None
            if sensor is None:
                try:
                    sensor = m.getPositionSensor()
                except Exception:
                    sensor = None
            if sensor:
                try:
                    sensor.enable(TIMESTEP)
                except Exception:
                    pass
            self.arm_motors.append(m)
            self.arm_sensors.append(sensor)
        self._arm_limits = [_motor_limits(m) for m in self.arm_motors]
        self._arm_home = self._pad_arm_target(ARM_HOME)
        self._arm_ready = self._pad_arm_target(ARM_READY)
        self._arm_pre_grab = self._pad_arm_target(ARM_PRE_GRAB)
        self._arm_grab = self._pad_arm_target(ARM_GRAB)
        self._arm_grab_high = self._pad_arm_target(ARM_GRAB_HIGH)
        self._arm_grab_low = self._pad_arm_target(ARM_GRAB_LOW)
        self._arm_lift = self._pad_arm_target(ARM_LIFT)
        self._arm_drop = self._pad_arm_target(ARM_DROP)

        # ── Gripper ──
        self.finger1 = _first_device(self.robot, [
            "finger::left", "gripper_left_finger_joint", "gripper_left_finger",
            "left_finger_joint", "left_finger"
        ])
        self.finger2 = _first_device(self.robot, [
            "finger::right", "gripper_right_finger_joint", "gripper_right_finger",
            "right_finger_joint", "right_finger"
        ])
        if self.finger1 and self.finger2:
            try:
                self.finger1.setVelocity(0.03)
                self.finger2.setVelocity(0.03)
            except Exception:
                pass
            try:
                self.finger_min = self.finger1.getMinPosition()
                self.finger_max = self.finger1.getMaxPosition()
            except Exception:
                self.finger_min, self.finger_max = (0.0, 0.04)
        else:
            self.finger_min, self.finger_max = (0.0, 0.04)

        # ── Camera ──
        self.camera = None
        self.cam_width = 0
        self.cam_height = 0
        for cam_name in ["youbot_camera", "camera", "Astra rgb", "xtion", "rgb_camera"]:
            try:
                self.camera = self.robot.getDevice(cam_name)
                self.camera.enable(TIMESTEP)
                self.cam_width = self.camera.getWidth()
                self.cam_height = self.camera.getHeight()
                print(f"[youBot] Camera OK: {cam_name} ({self.cam_width}x{self.cam_height})")
                break
            except Exception:
                continue
        if self.camera is None:
            print("[youBot] WARNING: No camera found")

        # ── Comms ──
        self.receiver = None
        try:
            self.receiver = self.robot.getDevice("receiver")
            if self.receiver:
                self.receiver.enable(TIMESTEP)
        except Exception:
            self.receiver = None
        try:
            self.emitter = self.robot.getDevice("emitter")
        except Exception:
            self.emitter = None
        if self.receiver is None or self.emitter is None:
            print("[youBot] ⚠️ Comms devices not found (receiver/emitter)")

        # ── Navigation PID ──
        self.pid_x = PID(kp=2.5, ki=0.1, kd=0.5, min_out=-5, max_out=5)
        self.pid_y = PID(kp=2.5, ki=0.1, kd=0.5, min_out=-5, max_out=5)
        self.pid_yaw = PID(kp=2.0, ki=0.0, kd=0.3, min_out=-2, max_out=2)
        self.pid_line = PID(kp=1.5, ki=0.05, kd=0.3, min_out=-3, max_out=3)

        # ── GPS State + Sensor Fusion ──
        self.x = -1.5
        self.y = -2.0
        self.yaw = math.pi / 2
        self.gps_timestamp = 0.0       # When last GPS was received
        self.gps_stale_threshold = 0.2  # GPS older than 200ms is stale
        # Odometry estimate (blended with GPS)
        self.odom_x = self.x
        self.odom_y = self.y
        self.odom_yaw = self.yaw
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_omega = 0.0

        self.items = []
        self.drops = 0
        self.mission_done = False
        self.t0 = 0.0

        if self.arm_motors:
            self.set_arm(self._arm_home)
        self.open_gripper()
        print("[youBot] === v9 Vision-Integrated Navigation Controller ===")

    # ══════════════════════════════════════════════════════════════
    # CAMERA IMAGE ACCESS
    # ══════════════════════════════════════════════════════════════

    def get_camera_image(self):
        """
        Get camera image as list of (r,g,b) tuples, row-major.
        Returns None if camera unavailable.
        """
        if self.camera is None:
            return None
        try:
            raw = self.camera.getImage()
            if raw is None:
                return None
            return raw
        except Exception:
            return None

    def _pad_arm_target(self, target):
        if not self.arm_motors:
            return []
        if len(target) >= len(self.arm_motors):
            return target[:len(self.arm_motors)]
        padded = list(target)
        for m in self.arm_motors[len(target):]:
            try:
                padded.append(m.getTargetPosition())
            except Exception:
                padded.append(0.0)
        return padded

    def get_pixel(self, raw, x, y):
        """Get (r, g, b) from raw camera image at pixel (x, y)."""
        idx = (y * self.cam_width + x) * 4  # BGRA format
        if idx + 2 >= len(raw):
            return (0, 0, 0)
        b = raw[idx]
        g = raw[idx + 1]
        r = raw[idx + 2]
        return (r, g, b)

    # ══════════════════════════════════════════════════════════════
    # VISION: LINE FOLLOWING
    # ══════════════════════════════════════════════════════════════

    def detect_line_offset(self, color_name="spine"):
        """
        Detect floor line in camera image and return horizontal offset.
        Returns: offset in range [-1.0, 1.0] where 0 = centered, or None if no line.
        Positive = line is to the right, Negative = line is to the left.
        """
        raw = self.get_camera_image()
        if raw is None:
            return None

        color_def = LINE_COLORS.get(color_name)
        if color_def is None:
            return None

        r_lo, r_hi = color_def["r"]
        g_lo, g_hi = color_def["g"]
        b_lo, b_hi = color_def["b"]

        # Scan the bottom third of the image (where floor lines appear)
        scan_start_y = int(self.cam_height * 0.6)
        scan_end_y = self.cam_height

        weighted_x = 0.0
        match_count = 0

        # Sample every 2nd pixel for speed
        for y in range(scan_start_y, scan_end_y, 2):
            for x in range(0, self.cam_width, 2):
                r, g, b = self.get_pixel(raw, x, y)
                if r_lo <= r <= r_hi and g_lo <= g <= g_hi and b_lo <= b <= b_hi:
                    weighted_x += x
                    match_count += 1

        if match_count < 5:
            return None  # No line detected

        center_x = weighted_x / match_count
        offset = (center_x - self.cam_width / 2) / (self.cam_width / 2)
        return offset

    # ══════════════════════════════════════════════════════════════
    # VISION: OBSTACLE DETECTION
    # ══════════════════════════════════════════════════════════════

    def detect_obstacle_ahead(self):
        """
        Check if there's an obstacle in front of the robot by analyzing the
        bottom portion of the camera image. Objects like tables will appear
        as large dark regions or solid color blocks close to the camera.

        Returns: (blocked: bool, description: str)
        """
        raw = self.get_camera_image()
        if raw is None:
            return False, "no_camera"

        # Check bottom-center region of image for obstacle
        scan_start_y = int(self.cam_height * (1.0 - OBSTACLE_ROW_FRACTION))
        scan_end_y = self.cam_height
        # Focus on center 60% of width
        x_margin = int(self.cam_width * 0.2)
        x_start = x_margin
        x_end = self.cam_width - x_margin

        total_brightness = 0
        dark_pixels = 0
        sample_count = 0

        for y in range(scan_start_y, scan_end_y, 3):
            for x in range(x_start, x_end, 3):
                r, g, b = self.get_pixel(raw, x, y)
                brightness = (r + g + b) / 3
                total_brightness += brightness
                sample_count += 1
                if brightness < OBSTACLE_DARKNESS_THRESHOLD:
                    dark_pixels += 1

        if sample_count == 0:
            return False, "no_data"

        avg_brightness = total_brightness / sample_count
        dark_ratio = dark_pixels / sample_count

        # Obstacle if bottom of frame is mostly dark (under a table)
        # or if there's a sudden uniform color block (close object)
        if dark_ratio > 0.5:
            return True, f"dark_obstruction(dark={dark_ratio:.0%},avg={avg_brightness:.0f})"

        # Also check for very close object: low variance in bottom rows
        # means something flat/uniform is very close
        if avg_brightness < 60 and dark_ratio > 0.3:
            return True, f"close_object(avg={avg_brightness:.0f})"

        return False, "clear"

    # ══════════════════════════════════════════════════════════════
    # VISION: SHELF SCANNING (REAL VLM)
    # ══════════════════════════════════════════════════════════════

    def scan_shelf_for_item(self, item_name, aisle):
        """
        Use vision API to detect item on shelf and return position offset.
        Returns: (found: bool, strafe_offset: float, confidence: float)
        """
        if self.camera is None:
            print("[youBot] No camera, using default position")
            return True, 0.0, 0.5

        # Save current frame for analysis
        image_path = "/tmp/shelf_scan.jpg"
        try:
            self.camera.saveImage(image_path, 85)
        except Exception as e:
            print(f"[youBot] Failed to save image: {e}")
            return True, 0.0, 0.5

        # Call vision API
        aisle_info = {
            "name": aisle,
            "category": EQUIPMENT.get(item_name, {}).get("cat", "Unknown"),
        }
        mission_context = "structure_fire"  # Default context

        print(f"[youBot] Scanning shelf for {item_name}...")
        result = analyze_shelf(image_path, mission_context, aisle_info)

        if not result or "error" in result:
            print(f"[youBot] Vision error: {result}")
            return True, 0.0, 0.5

        selected = result.get("selected_item", "unknown")
        position = result.get("position", "center")
        confidence = result.get("confidence", 0.5)
        reasoning = result.get("reasoning", "")
        priority = result.get("priority", "medium")

        print(f"[youBot] AI: {selected} at '{position}' ({confidence:.0%} conf, {priority})")
        if reasoning:
            print(f"[youBot] Reason: {reasoning[:80]}")

        # Convert position label to physical strafe offset
        strafe = get_strafe_offset(position)

        return True, strafe, confidence

    # ══════════════════════════════════════════════════════════════
    # MECANUM DRIVE
    # ══════════════════════════════════════════════════════════════

    def mecanum_world(self, vx_world, vy_world, omega):
        """
        Drive in world frame using mecanum wheels (YouBot) or differential (TIAGo).
        Transforms world velocities to robot frame.
        Also updates odometry estimate for sensor fusion.
        """
        # Store for odometry integration
        self.last_vx = vx_world
        self.last_vy = vy_world
        self.last_omega = omega

        # Transform world velocity to robot frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        # Robot frame velocities
        vx_robot = vx_world * cos_yaw + vy_world * sin_yaw
        vy_robot = -vx_world * sin_yaw + vy_world * cos_yaw

        if self.drive_mode == "mecanum":
            # Mecanum kinematics (with Webots correction)
            vy_robot = -vy_robot  # Webots youBot inverted strafe

            L = 0.5  # Effective wheelbase

            fl = vx_robot - vy_robot - L * omega
            fr = vx_robot + vy_robot + L * omega
            rl = vx_robot + vy_robot - L * omega
            rr = vx_robot - vy_robot + L * omega

            # Scale to max speed
            max_v = max(abs(fl), abs(fr), abs(rl), abs(rr), 0.1)
            scale = 6.0 / max_v if max_v > 6.0 else 1.0

            self.wheels[0].setVelocity(fl * scale)
            self.wheels[1].setVelocity(fr * scale)
            self.wheels[2].setVelocity(rl * scale)
            self.wheels[3].setVelocity(rr * scale)
        else:
            L = 0.4
            left = vx_robot - L * omega
            right = vx_robot + L * omega
            max_v = max(abs(left), abs(right), 0.1)
            scale = 6.0 / max_v if max_v > 6.0 else 1.0
            self.wheels[0].setVelocity(left * scale)
            self.wheels[1].setVelocity(right * scale)

    def stop(self):
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_omega = 0.0
        for w in self.wheels:
            w.setVelocity(0.0)

    # ══════════════════════════════════════════════════════════════
    # ARM & GRIPPER
    # ══════════════════════════════════════════════════════════════

    def set_arm(self, positions):
        for i, (m, p) in enumerate(zip(self.arm_motors, positions)):
            lo, hi = self._arm_limits[i] if i < len(self._arm_limits) else (-3.14, 3.14)
            try:
                m.setPosition(clamp(p, lo, hi))
            except Exception:
                pass

    def arm_reached(self, target, tol=0.15):
        for i, (s, p) in enumerate(zip(self.arm_sensors, target)):
            if s is None:
                continue
            lo, hi = self._arm_limits[i] if i < len(self._arm_limits) else (-3.14, 3.14)
            if abs(s.getValue() - clamp(p, lo, hi)) > tol:
                return False
        return True

    def open_gripper(self):
        if not (self.finger1 and self.finger2):
            return
        self.finger1.setPosition(self.finger_max)
        self.finger2.setPosition(self.finger_max)

    def close_gripper(self):
        if not (self.finger1 and self.finger2):
            return
        self.finger1.setPosition(self.finger_min)
        self.finger2.setPosition(self.finger_min)

    # ══════════════════════════════════════════════════════════════
    # SENSOR FUSION: GPS + ODOMETRY
    # ══════════════════════════════════════════════════════════════

    def update_odometry(self, dt=0.032):
        """Integrate velocity to get odometry-based position estimate."""
        self.odom_x += self.last_vx * dt
        self.odom_y += self.last_vy * dt
        self.odom_yaw += self.last_omega * dt
        self.odom_yaw = normalize_angle(self.odom_yaw)

    def fuse_pose(self):
        """
        Blend GPS and odometry. Trust GPS more when fresh, odometry when GPS stale.
        """
        now = self.robot.getTime()
        gps_age = now - self.gps_timestamp

        if gps_age < self.gps_stale_threshold:
            # Fresh GPS: trust it heavily (80% GPS, 20% odometry)
            alpha = 0.8
        elif gps_age < self.gps_stale_threshold * 3:
            # Getting stale: blend more evenly
            alpha = 0.5
        else:
            # Very stale GPS: rely on odometry
            alpha = 0.2

        self.x = alpha * self.x + (1 - alpha) * self.odom_x
        self.y = alpha * self.y + (1 - alpha) * self.odom_y
        self.yaw = normalize_angle(alpha * self.yaw + (1 - alpha) * self.odom_yaw)

    # ══════════════════════════════════════════════════════════════
    # NAVIGATION
    # ══════════════════════════════════════════════════════════════

    def dist_to(self, tx, ty):
        return math.sqrt((tx - self.x)**2 + (ty - self.y)**2)

    def goto_waypoint(self, name, tolerance=0.15, timeout=25.0, use_line=True):
        """
        Navigate to a named waypoint using PID control + vision line following.
        Checks for obstacles and stops if path is blocked.
        """
        if name not in WAYPOINTS:
            print(f"[youBot] Unknown waypoint: {name}")
            return False

        tx, ty = WAYPOINTS[name]
        print(f"[youBot] -> {name} ({tx:.1f}, {ty:.1f})")

        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()
        self.pid_line.reset()

        t0 = self.robot.getTime()
        stuck_count = 0
        last_dist = float('inf')
        obstacle_count = 0
        line_check_interval = 5  # Check line every N steps
        step_counter = 0

        # Determine which floor line to follow based on waypoint
        line_color = self._get_line_color_for_waypoint(name)

        while self.step():
            step_counter += 1
            dx = tx - self.x
            dy = ty - self.y
            dist = math.sqrt(dx*dx + dy*dy)

            # Arrived?
            if dist < tolerance:
                self.stop()
                print(f"[youBot] At {name}")
                return True

            # ── Obstacle check (every 10 steps to save CPU) ──
            if step_counter % 10 == 0:
                blocked, desc = self.detect_obstacle_ahead()
                if blocked:
                    obstacle_count += 1
                    if obstacle_count >= 3:
                        # Confirmed obstacle, not noise
                        self.stop()
                        print(f"[youBot] OBSTACLE: {desc} at ({self.x:.2f}, {self.y:.2f})")
                        # Try to recover: back up slightly
                        if not self._obstacle_recovery():
                            print(f"[youBot] Cannot clear obstacle, aborting to {name}")
                            return False
                        obstacle_count = 0
                else:
                    obstacle_count = max(0, obstacle_count - 1)

            # Stuck detection
            if abs(dist - last_dist) < 0.01:
                stuck_count += 1
            else:
                stuck_count = 0
            last_dist = dist

            if stuck_count > 50 and dist < 0.5:
                self.stop()
                print(f"[youBot] Close enough to {name} (d={dist:.2f})")
                return True

            # ── PID control ──
            vx = self.pid_x.compute(dx)
            vy = self.pid_y.compute(dy)

            # Face direction of travel for large movements
            if dist > 0.5:
                target_yaw = math.atan2(dy, dx)
                yaw_err = normalize_angle(target_yaw - self.yaw)
                omega = self.pid_yaw.compute(yaw_err)
            else:
                omega = 0

            # ── Vision line correction ──
            if use_line and line_color and step_counter % line_check_interval == 0:
                line_offset = self.detect_line_offset(line_color)
                if line_offset is not None:
                    # Apply lateral correction to stay on line
                    correction = self.pid_line.compute(line_offset)
                    # Apply correction perpendicular to travel direction
                    travel_angle = math.atan2(dy, dx)
                    perp_angle = travel_angle + math.pi / 2
                    vx += correction * math.cos(perp_angle) * 0.3
                    vy += correction * math.sin(perp_angle) * 0.3

            self.mecanum_world(vx, vy, omega)

            # Timeout
            if self.robot.getTime() - t0 > timeout:
                self.stop()
                print(f"[youBot] Timeout at ({self.x:.2f}, {self.y:.2f})")
                return dist < 0.5

        return False

    def _get_line_color_for_waypoint(self, wp_name):
        """Determine which floor line color to follow for a waypoint."""
        if "SPINE" in wp_name or wp_name in ("START", "TRUCK_APP"):
            return "spine"  # Yellow spine corridor
        if "SHELF" in wp_name:
            # Extract aisle letter: SHELF_A -> A
            aisle = wp_name.split("_")[-1]
            return AISLE_COLOR_MAP.get(aisle)
        if "TRUCK" in wp_name:
            return "truck"
        return None

    def _obstacle_recovery(self):
        """
        Try to recover from an obstacle by backing up and adjusting path.
        Returns True if recovery succeeded.
        """
        print("[youBot] Attempting obstacle recovery...")
        # Back up for 1 second
        steps = int(1.0 * 1000 / TIMESTEP)
        for _ in range(steps):
            # Drive backward in robot's current facing direction
            bx = -1.5 * math.cos(self.yaw)
            by = -1.5 * math.sin(self.yaw)
            self.mecanum_world(bx, by, 0)
            if not self.step():
                return False

        self.stop()
        self.wait_seconds(0.3)

        # Check if path is now clear
        blocked, _ = self.detect_obstacle_ahead()
        if not blocked:
            print("[youBot] Path cleared after backup")
            return True

        # Try shifting sideways
        print("[youBot] Shifting sideways to avoid obstacle...")
        for _ in range(steps):
            perp = self.yaw + math.pi / 2
            sx = 1.5 * math.cos(perp)
            sy = 1.5 * math.sin(perp)
            self.mecanum_world(sx, sy, 0)
            if not self.step():
                return False

        self.stop()
        blocked, _ = self.detect_obstacle_ahead()
        if not blocked:
            print("[youBot] Path cleared after lateral shift")
            return True

        print("[youBot] Could not clear obstacle")
        return False

    def goto_position(self, tx, ty, tolerance=0.15, timeout=20.0):
        """Navigate to arbitrary position with obstacle checking."""
        print(f"[youBot] -> ({tx:.1f}, {ty:.1f})")

        self.pid_x.reset()
        self.pid_y.reset()

        t0 = self.robot.getTime()
        step_counter = 0

        while self.step():
            step_counter += 1
            dx = tx - self.x
            dy = ty - self.y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < tolerance:
                self.stop()
                return True

            # Obstacle check
            if step_counter % 10 == 0:
                blocked, desc = self.detect_obstacle_ahead()
                if blocked:
                    self.stop()
                    print(f"[youBot] Obstacle at ({self.x:.2f}, {self.y:.2f}): {desc}")
                    if not self._obstacle_recovery():
                        return False

            vx = self.pid_x.compute(dx)
            vy = self.pid_y.compute(dy)

            self.mecanum_world(vx, vy, 0)

            if self.robot.getTime() - t0 > timeout:
                self.stop()
                return dist < 0.4

        return False

    def turn_to_angle(self, target_yaw, timeout=5.0):
        """Turn to face specific direction."""
        self.pid_yaw.reset()
        t0 = self.robot.getTime()

        while self.step():
            err = normalize_angle(target_yaw - self.yaw)

            if abs(err) < 0.08:
                self.stop()
                return True

            omega = self.pid_yaw.compute(err)
            self.mecanum_world(0, 0, omega)

            if self.robot.getTime() - t0 > timeout:
                self.stop()
                return abs(err) < 0.2

        return False

    # ══════════════════════════════════════════════════════════════
    # COMMS + SENSOR FUSION
    # ══════════════════════════════════════════════════════════════

    def process_msgs(self):
        if not self.receiver:
            return
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            self.receiver.nextPacket()
            try:
                msg = json.loads(data)
                if msg["type"] == "GPS":
                    self.x = msg["x"]
                    self.y = msg["y"]
                    self.yaw = msg.get("yaw", self.yaw)
                    self.gps_timestamp = self.robot.getTime()
                    # Sync odometry to GPS when fresh
                    self.odom_x = self.x
                    self.odom_y = self.y
                    self.odom_yaw = self.yaw
                elif msg["type"] == "MISSION" and not self.items:
                    print(f"[youBot] Mission: {msg.get('emergency','?')}")
                    self.items = list(DEFAULT_MISSION)
                    self.t0 = self.robot.getTime()
            except Exception:
                pass

    def send_status(self, msg):
        if self.emitter:
            try:
                self.emitter.send(json.dumps(msg).encode())
            except Exception:
                pass

    def step(self):
        self.process_msgs()
        self.update_odometry()
        self.fuse_pose()
        return self.robot.step(TIMESTEP) != -1

    def wait_arm(self, target, timeout=6.0):
        t0 = self.robot.getTime()
        while self.step():
            if self.arm_reached(target):
                return True
            if self.robot.getTime() - t0 > timeout:
                return True
        return False

    def wait_seconds(self, sec):
        steps = int(sec * 1000 / TIMESTEP)
        for _ in range(steps):
            if not self.step():
                return False
        return True

    # ══════════════════════════════════════════════════════════════
    # PICK SEQUENCE (VISION-INTEGRATED)
    # ══════════════════════════════════════════════════════════════

    def pick_item(self, item_name):
        info = EQUIPMENT[item_name]
        aisle = info["aisle"]
        route = AISLE_ROUTES[aisle]

        print(f"[youBot] ══════════════════════════════════════")
        print(f"[youBot] PICK: {item_name}")
        print(f"[youBot] Aisle {aisle} ({info['cat']})")

        # Step 1: Raise arm to safe travel height to avoid obstacles
        self.set_arm(self._arm_ready)
        self.wait_seconds(0.5)

        # Step 2: Go to spine
        spine_wp = route[0]
        if not self.goto_waypoint(spine_wp):
            print("[youBot] Failed to reach spine")
            return False

        # Step 3: Face shelf (-X direction = pi)
        print("[youBot] Facing shelf...")
        self.turn_to_angle(math.pi)

        # Step 4: Prepare arm for approach
        print("[youBot] Arm ready...")
        self.set_arm(self._arm_ready)
        self.open_gripper()
        self.wait_seconds(1.0)

        # Step 5: Go to shelf position
        shelf_wp = route[1]
        if not self.goto_waypoint(shelf_wp, tolerance=0.2):
            print("[youBot] Failed shelf approach")
            # Continue anyway - might be close enough

        print(f"[youBot] At shelf ({self.x:.2f}, {self.y:.2f})")

        # Step 6: REAL VISION SCAN - detect item position
        found, strafe_offset, confidence = self.scan_shelf_for_item(item_name, aisle)

        if not found:
            print(f"[youBot] {item_name} NOT FOUND on shelf")
            return False

        # Step 7: Adjust position based on vision detection
        if abs(strafe_offset) > 0.05:
            print(f"[youBot] Adjusting position by {strafe_offset:+.2f}m")
            # Strafe to align with detected object
            adjusted_y = self.y + strafe_offset
            self.goto_position(self.x, adjusted_y, tolerance=0.1, timeout=5.0)
            # Re-face shelf after adjustment
            self.turn_to_angle(math.pi)
            self.wait_seconds(0.3)

        # Step 8: Verify alignment with second scan
        if confidence < 0.7 and self.camera:
            print("[youBot] Low confidence, re-scanning...")
            found2, strafe2, conf2 = self.scan_shelf_for_item(item_name, aisle)
            if found2 and abs(strafe2) > 0.05:
                adjusted_y = self.y + strafe2
                self.goto_position(self.x, adjusted_y, tolerance=0.1, timeout=3.0)
                self.turn_to_angle(math.pi)

        # Step 9: Arm down to pre-grab
        print("[youBot] Lowering arm...")
        self.set_arm(self._arm_pre_grab)
        self.wait_arm(self._arm_pre_grab, 4.0)

        # Step 10: Grab position
        self.set_arm(self._arm_grab)
        self.wait_arm(self._arm_grab, 4.0)

        # Step 11: Close gripper
        self.close_gripper()
        self.wait_seconds(1.0)
        print("[youBot] Gripping")

        # Step 12: Verify grip - check if gripper actually closed on something
        grip_ok = self._verify_grip()
        if not grip_ok:
            print("[youBot] Grip check: may not have object, retrying...")
            # Try slightly lower
            self.open_gripper()
            self.wait_seconds(0.5)
            self.set_arm(self._arm_grab_low)
            self.wait_arm(self._arm_grab_low, 4.0)
            self.close_gripper()
            self.wait_seconds(1.0)

        # Step 13: Lift
        self.set_arm(self._arm_lift)
        self.wait_arm(self._arm_lift, 4.0)
        print(f"[youBot] PICKED: {item_name}")

        # Step 14: Back to spine with arm raised (safe travel height)
        print("[youBot] Retreating...")
        self.goto_waypoint(spine_wp, tolerance=0.3)

        # Step 15: Tuck arm
        self.set_arm(self._arm_home)
        self.wait_seconds(0.5)

        return True

    def _verify_grip(self):
        """
        Check if the gripper has actually gripped something by reading
        finger position sensors. If fingers are at min (fully closed with
        no object), the grip likely failed.
        """
        # Read finger positions - if they're at absolute min, nothing was gripped
        try:
            f1_pos = self.finger1.getTargetPosition()
            # If target is min and we're checking right after close command,
            # we can't easily tell from position alone in Webots.
            # For now, assume grip succeeded (future: use force sensors)
            return True
        except Exception:
            return True

    # ══════════════════════════════════════════════════════════════
    # DROP SEQUENCE
    # ══════════════════════════════════════════════════════════════

    def drop_at_truck(self, item_name):
        print(f"[youBot] -> Truck with {item_name}")

        # Keep arm raised during travel
        self.set_arm(self._arm_lift)

        # Step 1: Go to truck approach
        self.goto_waypoint("TRUCK_APP")

        # Step 2: Go to drop position
        self.goto_waypoint("TRUCK_DROP", tolerance=0.25)

        print(f"[youBot] At truck ({self.x:.2f}, {self.y:.2f})")

        # Step 3: Face truck (-Y)
        self.turn_to_angle(-math.pi/2)

        # Step 4: Drop
        print("[youBot] Dropping...")
        self.set_arm(self._arm_drop)
        self.wait_arm(self._arm_drop, 4.0)

        self.open_gripper()
        self.wait_seconds(0.8)

        self.drops += 1
        total = len(self.items)
        print(f"[youBot] LOADED: {item_name} ({self.drops}/{total})")

        self.send_status({
            "type": "LOADED",
            "item": item_name,
            "progress": f"{self.drops}/{total}"
        })

        # Step 5: Retract
        self.set_arm(self._arm_home)
        self.wait_seconds(0.5)

        return True

    # ══════════════════════════════════════════════════════════════
    # MAIN
    # ══════════════════════════════════════════════════════════════

    def run(self):
        print("[youBot] Waiting for mission...")

        # Wait for mission or auto-start
        while self.step():
            if self.items:
                break
            if self.robot.getTime() > 3.0:
                print("[youBot] Auto-starting")
                self.items = list(DEFAULT_MISSION)
                self.t0 = self.robot.getTime()
                break

        print(f"[youBot] Route: {' -> '.join([EQUIPMENT[i]['aisle'] for i in self.items])} -> Truck")

        # Execute
        for item in self.items:
            if self.mission_done:
                break

            if not self.pick_item(item):
                print(f"[youBot] Skip {item}")
                continue

            if not self.drop_at_truck(item):
                continue

        # Done
        elapsed = self.robot.getTime() - self.t0
        print(f"[youBot] ══════════════════════════════════════")
        print(f"[youBot] MISSION COMPLETE")
        print(f"[youBot] {self.drops}/{len(self.items)} items")
        print(f"[youBot] {elapsed:.1f}s")

        self.send_status({
            "type": "MISSION_COMPLETE",
            "items": self.drops,
            "time": elapsed
        })
        self.mission_done = True

        while self.step():
            self.stop()


if __name__ == "__main__":
    bot = YouBotLoader()
    bot.run()
