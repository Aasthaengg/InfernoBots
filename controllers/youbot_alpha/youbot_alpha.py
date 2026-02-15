#!/usr/bin/env python3
"""
YouBot ALPHA Controller — VLM-Enabled Multi-Robot System

Features:
  - Moondream VLM shelf scanning
  - Object verification before pick
  - Obstacle detection
  - Status reporting to supervisor
"""

from controller import Robot
import json
import math
import sys
import os
import numpy as np

TIMESTEP = 32
ROBOT_ID = "alpha"

# Add project paths for imports
PROJECT_ROOT = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, PROJECT_ROOT)

# Try to import VLM modules
try:
    from perception.vlm_moondream import get_vlm
    VLM_AVAILABLE = True
except ImportError:
    VLM_AVAILABLE = False
    print(f"[{ROBOT_ID.upper()}] ⚠️ VLM modules not found, using mock")


class PID:
    def __init__(self, kp, ki, kd, min_out=-10, max_out=10):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.min_out, self.max_out = min_out, max_out
        self.integral = 0
        self.prev_error = 0
    
    def compute(self, error, dt=0.032):
        self.integral = max(-5, min(5, self.integral + error * dt))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(self.min_out, min(self.max_out, output))
    
    def reset(self):
        self.integral = self.prev_error = 0


# Arm configuration (YouBot defaults; TIAGo will be clamped to its limits)
ARM_SPEEDS = [0.5, 0.8, 0.8, 0.6, 0.3]
ARM_HOME = [0.0, -1.13, -2.54, -1.78, 0.0]
ARM_READY = [0.0, -0.50, -1.80, -1.30, 0.0]
ARM_PRE_GRAB = [0.0, 0.10, -1.10, -1.60, 0.0]
ARM_GRAB = [0.0, 0.40, -0.70, -1.80, 0.0]
ARM_LIFT = [0.0, -0.70, -2.00, -1.00, 0.0]
ARM_DROP = [0.0, -0.10, -1.20, -1.50, 0.0]
ARM_LIMITS = [(-2.949, 2.949), (-1.1345, 1.5708), (-2.5481, 2.5481), (-1.7802, 1.7802), (-2.949, 2.949)]

# Waypoints
AISLE_Y = {"A": 5.5, "B": 3.5, "C": 1.5, "D": -0.5}
SPINE_X = -1.5
SHELF_X = -3.4
TRUCK_APP = (-1.5, -3.0)
TRUCK_DROP = (0.0, -4.2)

# Line-following settings (camera-based)
LINE_FOLLOW_SPEED = 2.0
LINE_FOLLOW_TURN = 1.2
LINE_LOST_TIMEOUT = 1.5


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


class YouBotAlpha:
    def __init__(self):
        self.robot = Robot()
        
        # Wheels (YouBot mecanum or TIAGo differential)
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
        
        # Arm
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
        self._arm_lift = self._pad_arm_target(ARM_LIFT)
        self._arm_drop = self._pad_arm_target(ARM_DROP)
        
        # Gripper
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
        
        # Camera for VLM
        self.camera = None
        for cam_name in ["camera", "Astra rgb", "xtion", "rgb_camera"]:
            try:
                self.camera = self.robot.getDevice(cam_name)
                if self.camera:
                    self.camera.enable(TIMESTEP)
                    self.cam_width = self.camera.getWidth()
                    self.cam_height = self.camera.getHeight()
                    print(f"[{ROBOT_ID.upper()}] 📷 Camera: {cam_name} {self.cam_width}x{self.cam_height}")
                    break
            except Exception:
                continue
        
        # VLM
        if VLM_AVAILABLE:
            self.vlm = get_vlm(use_mock=False)  # Set False for real Ollama
        else:
            self.vlm = None
        
        # Comms
        self.receiver = None
        self.emitter = None
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
            print(f"[{ROBOT_ID.upper()}] ⚠️ Comms devices not found (receiver/emitter)")
        
        # Navigation
        self.pid_x = PID(2.5, 0.1, 0.5, -5, 5)
        self.pid_y = PID(2.5, 0.1, 0.5, -5, 5)
        self.pid_yaw = PID(2.0, 0.0, 0.3, -2, 2)
        
        # State
        self.x, self.y, self.yaw = -1.0, -2.0, math.pi/2
        self.current_task = None
        self.carrying = None
        self.waiting = False
        self.mission_active = False
        self._last_debug = 0.0
        self._last_status = 0.0
        
        if self.arm_motors:
            self.set_arm(self._arm_home)
        if self.finger1 and self.finger2:
            self.open_gripper()
        
        print(f"[{ROBOT_ID.upper()}] ═══ VLM Controller Ready (Blue) ═══")
    
    def get_camera_frame(self):
        """Get frame from camera for VLM."""
        if not self.camera:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        image = self.camera.getImage()
        if image is None:
            return np.zeros((self.cam_height, self.cam_width, 3), dtype=np.uint8)
        
        frame = np.frombuffer(image, np.uint8).reshape((self.cam_height, self.cam_width, 4))
        return frame[:, :, :3].copy()

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

    def detect_line_offset(self, target: str):
        """Return normalized x offset of target color line in bottom strip."""
        if not self.camera:
            return None
        frame = self.get_camera_frame()
        h, w, _ = frame.shape
        strip = frame[int(h * 0.75):, :, :]

        r = strip[:, :, 0].astype(np.int16)
        g = strip[:, :, 1].astype(np.int16)
        b = strip[:, :, 2].astype(np.int16)

        if target == "yellow":
            mask = (r > 180) & (g > 180) & (b < 80)
        elif target == "red":
            mask = (r > 180) & (g < 80) & (b < 80)
        elif target == "green":
            mask = (g > 160) & (r < 80) & (b < 80)
        elif target == "blue":
            mask = (b > 160) & (r < 80) & (g < 80)
        elif target == "orange":
            mask = (r > 180) & (g > 90) & (g < 160) & (b < 80)
        else:
            return None

        ys, xs = np.where(mask)
        if xs.size < 30:
            return None
        x_mean = xs.mean()
        offset = (x_mean / w) * 2.0 - 1.0  # -1 left, +1 right
        return float(offset)

    def follow_line_to_y(self, target_y: float, line_color: str = "yellow", tol: float = 0.15):
        """Follow a colored floor line while moving to a target Y."""
        t0 = self.robot.getTime()
        last_seen = t0
        while self.step():
            if self.waiting:
                self.stop()
                return False

            if abs(self.y - target_y) <= tol:
                self.stop()
                return True

            offset = self.detect_line_offset(line_color)
            if offset is not None:
                last_seen = self.robot.getTime()
                omega = -offset * LINE_FOLLOW_TURN
                self.mecanum_world(0.0, LINE_FOLLOW_SPEED, omega)
            else:
                if self.robot.getTime() - last_seen > LINE_LOST_TIMEOUT:
                    return self.goto(SPINE_X, target_y, tolerance=tol)
                self.mecanum_world(0.0, LINE_FOLLOW_SPEED * 0.6, 0.0)

            if self.vlm and int(self.robot.getTime() * 10) % 10 == 0:
                obs = self.vlm.check_obstacles(self.get_camera_frame())
                if obs.get("blocked"):
                    self.send({"type": "STATUS", "state": "blocked", "detail": obs})
                    self.stop()
                    self.wait_time(0.5)
        return False
    
    def vlm_scan_shelf(self, target_item: str = None):
        """Scan shelf with VLM."""
        if not self.vlm:
            return {"objects": [], "target_visible": True, "confidence": 0.8}
        
        frame = self.get_camera_frame()
        
        if target_item:
            result = self.vlm.verify_object(frame, target_item)
            return {
                "objects": [target_item] if result.get("found") else [],
                "target_visible": result.get("found", False),
                "confidence": result.get("confidence", 0.0)
            }
        else:
            return self.vlm.analyze(frame, prompt_type="shelf_scan")
    
    def mecanum_world(self, vx, vy, omega):
        if self.drive_mode == "mecanum":
            cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
            vx_r = vx * cos_yaw + vy * sin_yaw
            vy_r = -(-vx * sin_yaw + vy * cos_yaw)
            
            L = 0.5
            fl = vx_r - vy_r - L * omega
            fr = vx_r + vy_r + L * omega
            rl = vx_r + vy_r - L * omega
            rr = vx_r - vy_r + L * omega
            
            max_v = max(abs(fl), abs(fr), abs(rl), abs(rr), 0.1)
            scale = 6.0 / max_v if max_v > 6.0 else 1.0
            
            self.wheels[0].setVelocity(fl * scale)
            self.wheels[1].setVelocity(fr * scale)
            self.wheels[2].setVelocity(rl * scale)
            self.wheels[3].setVelocity(rr * scale)
        else:
            # Differential drive fallback (TIAGo)
            cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
            vx_r = vx * cos_yaw + vy * sin_yaw
            forward = vx_r
            L = 0.4
            left = forward - L * omega
            right = forward + L * omega
            max_v = max(abs(left), abs(right), 0.1)
            scale = 6.0 / max_v if max_v > 6.0 else 1.0
            self.wheels[0].setVelocity(left * scale)
            self.wheels[1].setVelocity(right * scale)
    
    def stop(self):
        for w in self.wheels:
            w.setVelocity(0.0)
    
    def set_arm(self, pos):
        for i, (m, p) in enumerate(zip(self.arm_motors, pos)):
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
    
    def send(self, msg: dict):
        msg["_from"] = ROBOT_ID
        if self.emitter:
            self.emitter.send(json.dumps(msg).encode())

    def process_messages(self):
        if not self.receiver:
            return
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            self.receiver.nextPacket()
            
            try:
                msg = json.loads(data)
                if msg.get("_to") and msg["_to"] != ROBOT_ID:
                    continue
                
                msg_type = msg.get("type", "")
                
                if msg_type == "GPS":
                    self.x, self.y = msg["x"], msg["y"]
                    self.yaw = msg.get("yaw", self.yaw)
                    if self.robot.getTime() - self._last_debug > 2.0:
                        print(f"[{ROBOT_ID.upper()}][DEBUG] GPS x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.2f}")
                elif msg_type == "TASK":
                    self.current_task = msg
                    self.waiting = False
                    print(f"[{ROBOT_ID.upper()}] 📋 Task: {msg.get('action')} {msg.get('item')}")
                elif msg_type == "WAIT":
                    self.waiting = True
                    print(f"[{ROBOT_ID.upper()}] ⏳ Waiting...")
                elif msg_type == "MISSION_START":
                    self.mission_active = True
                    print(f"[{ROBOT_ID.upper()}] 🚨 Mission active")
            except:
                pass
    
    def step(self):
        self.process_messages()
        return self.robot.step(TIMESTEP) != -1
    
    def wait_arm(self, target, timeout=5.0):
        t0 = self.robot.getTime()
        while self.step():
            if self.arm_reached(target) or self.robot.getTime() - t0 > timeout:
                return True
        return False
    
    def wait_time(self, sec):
        steps = int(sec * 1000 / TIMESTEP)
        for _ in range(steps):
            if not self.step():
                return False
        return True
    
    def goto(self, tx, ty, tolerance=0.15, timeout=25.0):
        self.pid_x.reset()
        self.pid_y.reset()
        
        t0 = self.robot.getTime()
        stuck = 0
        last_d = float('inf')
        
        while self.step():
            if self.waiting:
                self.stop()
                return False
            
            dx, dy = tx - self.x, ty - self.y
            d = math.sqrt(dx*dx + dy*dy)
            
            if d < tolerance:
                self.stop()
                return True
            
            if abs(d - last_d) < 0.01:
                stuck += 1
            else:
                stuck = 0
            last_d = d
            
            if stuck > 40 and d < 0.5:
                self.stop()
                return True
            
            vx = self.pid_x.compute(dx)
            vy = self.pid_y.compute(dy)
            omega = self.pid_yaw.compute(normalize_angle(math.atan2(dy, dx) - self.yaw)) if d > 0.5 else 0
            
            self.mecanum_world(vx, vy, omega)
            
            if self.robot.getTime() - t0 > timeout:
                self.stop()
                return d < 0.5
        return False
    
    def turn_to(self, target_yaw, timeout=4.0):
        self.pid_yaw.reset()
        t0 = self.robot.getTime()
        
        while self.step():
            err = normalize_angle(target_yaw - self.yaw)
            if abs(err) < 0.08:
                self.stop()
                return True
            self.mecanum_world(0, 0, self.pid_yaw.compute(err))
            if self.robot.getTime() - t0 > timeout:
                self.stop()
                return True
        return False
    
    def execute_pick(self, item: str, aisle: str, task_id: str) -> bool:
        y = AISLE_Y[aisle]
        print(f"[{ROBOT_ID.upper()}] → Aisle {aisle}")
        
        # Go to spine
        if not self.follow_line_to_y(y, line_color="yellow"):
            return False
        
        self.turn_to(math.pi)
        self.set_arm(self._arm_ready)
        self.open_gripper()
        self.wait_time(0.5)
        
        # Go to shelf
        if not self.goto(SHELF_X, y, tolerance=0.35):
            return False
        
        print(f"[{ROBOT_ID.upper()}] 📍 At shelf")
        
        # ═══ VLM SCAN with micro-adjustments ═══
        print(f"[{ROBOT_ID.upper()}] 👁️ VLM scanning for {item}...")
        vlm_result = None
        offsets = [(0.0, 0.0), (0.15, 0.0), (0.15, 0.10), (0.15, -0.10)]
        for dx, dy in offsets:
            if dx != 0.0 or dy != 0.0:
                self.goto(SHELF_X + dx, y + dy, tolerance=0.30, timeout=6.0)
            self.turn_to(math.pi)
            vlm_result = self.vlm_scan_shelf(target_item=item)
            if vlm_result.get("target_visible", True):
                break
        
        # Report to supervisor
        self.send({
            "type": "VLM_RESULT",
            "aisle": aisle,
            "result": vlm_result
        })
        
        if not vlm_result.get("target_visible", True):
            print(f"[{ROBOT_ID.upper()}] ❌ Item not visible!")
            self.send({"type": "FAILED", "task_id": task_id, "reason": "item_not_found"})
            self.goto(SPINE_X, y)
            return False
        
        print(f"[{ROBOT_ID.upper()}] ✓ Item verified (conf: {vlm_result.get('confidence', 0):.2f})")
        
        # Pick sequence
        self.set_arm(self._arm_pre_grab)
        self.wait_arm(self._arm_pre_grab, 3.0)
        self.set_arm(self._arm_grab)
        self.wait_arm(self._arm_grab, 3.0)
        self.close_gripper()
        self.wait_time(0.8)
        self.set_arm(self._arm_lift)
        self.wait_arm(self._arm_lift, 3.0)
        
        self.carrying = item
        print(f"[{ROBOT_ID.upper()}] ✋ Picked: {item}")
        self.send({"type": "PICKED", "item": item, "task_id": task_id})
        
        self.goto(SPINE_X, y, tolerance=0.3)
        self.set_arm(self._arm_home)
        
        return True
    
    def execute_deliver(self, item: str, task_id: str) -> bool:
        print(f"[{ROBOT_ID.upper()}] → Truck")
        
        if not self.goto(TRUCK_APP[0], TRUCK_APP[1]):
            return False
        
        if not self.goto(TRUCK_DROP[0], TRUCK_DROP[1], tolerance=0.25):
            return False
        
        print(f"[{ROBOT_ID.upper()}] 🚛 At truck")
        self.turn_to(-math.pi / 2)
        
        self.set_arm(self._arm_drop)
        self.wait_arm(self._arm_drop, 3.0)
        self.open_gripper()
        self.wait_time(0.6)
        
        print(f"[{ROBOT_ID.upper()}] 📦 Delivered: {item}")
        self.send({"type": "DELIVERED", "item": item, "task_id": task_id})
        
        self.carrying = None
        self.set_arm(self._arm_home)
        self.wait_time(0.3)
        
        return True
    
    def run(self):
        print(f"[{ROBOT_ID.upper()}] ⏳ Waiting for mission...")
        
        while self.step():
            now = self.robot.getTime()
            if now - self._last_status > 1.0:
                self.send({
                    "type": "STATUS",
                    "state": "active" if self.mission_active else "idle",
                    "position": [self.x, self.y],
                    "carrying": self.carrying,
                    "task": self.current_task.get("task_id") if self.current_task else None
                })
                self._last_status = now
            if now - self._last_debug > 2.0:
                print(f"[{ROBOT_ID.upper()}][DEBUG] t={now:.1f} mission={self.mission_active} waiting={self.waiting} task={'yes' if self.current_task else 'no'} pos=({self.x:.2f},{self.y:.2f})")
                self._last_debug = now
            if not self.mission_active:
                continue
            
            if self.current_task and not self.waiting:
                task = self.current_task
                action = task.get("action")
                item = task.get("item")
                task_id = task.get("task_id")
                
                self.current_task = None
                
                if action == "pick":
                    aisle = task.get("aisle")
                    success = self.execute_pick(item, aisle, task_id)
                    if not success:
                        self.send({"type": "FAILED", "task_id": task_id})
                
                elif action == "deliver":
                    success = self.execute_deliver(item, task_id)
                    if not success:
                        self.send({"type": "FAILED", "task_id": task_id})
            
            elif self.waiting:
                self.stop()


if __name__ == "__main__":
    bot = YouBotAlpha()
    bot.run()
