#!/usr/bin/env python3
"""
FireBot Supervisor — Alpha-Only Pick & Place
Navigate to shelf aisle → grab box → navigate to drop → place box
"""

from controller import Supervisor
import json
import math
import sys
import os
import random
import socket

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, "/home/aastha/firebot")
_conda_site = "/home/aastha/miniforge3/lib/python3.12/site-packages"
if os.path.isdir(_conda_site) and _conda_site not in sys.path:
    sys.path.insert(0, _conda_site)

from multi_robot_graph import MultiRobotGraph, Phase
from config.settings import WAYPOINTS, get_aisle_for_item

TIMESTEP = 32
SPINE_X = -1.5
WAYPOINT_REACHED_DIST = 0.40

LOG_PATH = "/home/aastha/firebot/supervisor.log"
_logfile = open(LOG_PATH, "w", buffering=1)

def log(msg):
    print(msg, flush=True)
    _logfile.write(msg + "\n")

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def normalize_angle(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def plan_pick_path(current_pos, aisle):
    spine_wp = WAYPOINTS.get(f"SPINE_{aisle}")
    shelf_wp = WAYPOINTS.get(f"SHELF_{aisle}")
    if not spine_wp or not shelf_wp:
        return [shelf_wp or (-2.0, 0.0)]
    return [(SPINE_X, current_pos[1]), spine_wp, shelf_wp]

def plan_deliver_path(current_pos, current_aisle):
    spine_wp = WAYPOINTS.get(f"SPINE_{current_aisle}")
    truck_approach = WAYPOINTS.get("TRUCK_APPROACH", (-1.5, -3.0))
    truck_drop = WAYPOINTS.get("TRUCK_DROP", (0.0, -4.2))
    path = []
    if spine_wp:
        path.append(spine_wp)
    path.append(truck_approach)
    path.append(truck_drop)
    return path


# Drop positions for delivered items (so they don't stack)
DROP_OFFSETS = [(0.0, 0.0), (0.3, 0.0), (0.0, 0.3), (0.3, 0.3)]


class FireBotSupervisor:
    def __init__(self):
        self.supervisor = Supervisor()
        self.youbot_alpha = self.supervisor.getFromDef("YOUBOT_ALPHA")
        log(f"[Init] ALPHA: {'OK' if self.youbot_alpha else 'MISSING'}")

        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(TIMESTEP)

        self.graph = MultiRobotGraph(timeout_s=1.5)

        self.mission_started = False
        self.last_commands = {}
        self._last_debug = 0.0

        # Navigation
        self.nav_waypoints = []
        self.nav_phase = "idle"
        self.nav_aisle = None
        self.carrying = None         # item name string
        self.carried_node = None     # Webots node of the box being carried
        self.items_dropped = 0

        # Stuck detection
        self._stuck_since = 0.0
        self._stuck_ref = [0, 0]
        self._stuck_yaw_ref = 0.0

        # UDP to ROS2 nav_bridge
        self.ros_bridge_addr = ("127.0.0.1", int(os.environ.get("FIREBOT_ROS_UDP_PORT", "9909")))
        self.ros_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        log("[Init] Ready")

    def get_robot_pos(self):
        if not self.youbot_alpha:
            return None, None, None
        pos = self.youbot_alpha.getPosition()
        rot = self.youbot_alpha.getOrientation()
        yaw = math.atan2(rot[3], rot[0])
        return pos[0], pos[1], yaw

    def send_cmd(self, linear, angular):
        payload = json.dumps({
            "robot": "alpha",
            "cmd_vel": {"linear": float(linear), "angular": float(angular)}
        }).encode()
        try:
            self.ros_sock.sendto(payload, self.ros_bridge_addr)
        except Exception:
            pass

    # ── Box handling (supervisor moves objects) ──────────────

    def grab_box(self, item_name):
        """Find the box by DEF name and start carrying it."""
        node = self.supervisor.getFromDef(item_name)
        if node:
            self.carried_node = node
            log(f"[Box] Grabbed '{item_name}' node")
        else:
            log(f"[Box] WARNING: no node with DEF '{item_name}' found in world")
            self.carried_node = None

    def move_box_with_robot(self, rx, ry):
        """Keep the carried box above the robot."""
        if not self.carried_node:
            return
        try:
            tf = self.carried_node.getField("translation")
            tf.setSFVec3f([rx, ry, 1.1])  # float above robot
        except Exception as e:
            log(f"[Box] move error: {e}")

    def place_box(self):
        """Place the box on the truck platform table."""
        if not self.carried_node:
            return
        try:
            # Table is at (0, -4.5), top surface at z=0.3, box sits on top
            idx = self.items_dropped % len(DROP_OFFSETS)
            ox, oy = DROP_OFFSETS[idx]
            tx, ty = 0.0 + ox, -4.5 + oy
            tf = self.carried_node.getField("translation")
            tf.setSFVec3f([tx, ty, 0.50])
            log(f"[Box] Placed on table at ({tx:.1f}, {ty:.1f})")
        except Exception as e:
            log(f"[Box] place error: {e}")
        self.carried_node = None
        self.items_dropped += 1

    # ── Navigation ────────────────────────────────────────────

    def navigate(self, x, y, yaw):
        if not self.nav_waypoints:
            self.send_cmd(0.0, 0.0)
            return False

        now = self.supervisor.getTime()

        # Stuck detection
        if self._stuck_since == 0.0:
            self._stuck_since = now
            self._stuck_ref = [x, y]
            self._stuck_yaw_ref = yaw
        else:
            elapsed = now - self._stuck_since
            if elapsed > 8.0:
                d = math.sqrt((x - self._stuck_ref[0])**2 + (y - self._stuck_ref[1])**2)
                yaw_change = abs(normalize_angle(yaw - self._stuck_yaw_ref))
                if d < 0.3 and yaw_change < 0.3:
                    log(f"[Nav] STUCK 8s (d={d:.2f}m rot={yaw_change:.2f}), skip wp")
                    self.nav_waypoints.pop(0)
                    self._stuck_since = 0.0
                    return len(self.nav_waypoints) == 0
                else:
                    self._stuck_since = now
                    self._stuck_ref = [x, y]
                    self._stuck_yaw_ref = yaw

        target = self.nav_waypoints[0]
        dx = target[0] - x
        dy = target[1] - y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < WAYPOINT_REACHED_DIST:
            self.nav_waypoints.pop(0)
            if self.nav_waypoints:
                log(f"[Nav] WP reached, next: {self.nav_waypoints[0]} ({len(self.nav_waypoints)} left)")
                return False
            else:
                self.send_cmd(0.0, 0.0)
                return True

        target_yaw = math.atan2(dy, dx)
        err = normalize_angle(target_yaw - yaw)

        if abs(err) > 1.2:
            self.send_cmd(0.0, clamp(2.0 * err, -1.0, 1.0))
        elif abs(err) > 0.5:
            self.send_cmd(clamp(0.3 * dist, 0.0, 0.15), clamp(2.0 * err, -1.2, 1.2))
        else:
            self.send_cmd(clamp(0.8 * dist, 0.0, 0.45), clamp(1.5 * err, -1.0, 1.0))
        return False

    # ── Main loop ─────────────────────────────────────────────

    def run(self):
        last_graph = 0.0
        pick_time = 0.0
        drop_time = 0.0
        current_item = None

        log("[Run] Main loop started")

        while self.supervisor.step(TIMESTEP) != -1:
            now = self.supervisor.getTime()
            x, y, yaw = self.get_robot_pos()
            if x is None:
                continue

            self.graph.update_robot("alpha", {
                "position": [x, y], "yaw": yaw, "last_update": now
            })

            # Keep carried box with robot
            if self.carried_node:
                self.move_box_with_robot(x, y)

            # Debug
            if now - self._last_debug > 3.0:
                log(f"[t={now:.0f}] ({x:.2f},{y:.2f}) phase={self.nav_phase} carry={self.carrying} wps={len(self.nav_waypoints)}")
                self._last_debug = now

            # Start mission
            if not self.mission_started and now > 2.0:
                emergency = random.choice(["wildfire", "electrical_fire", "chemical_spill"])
                log(f"[Mission] {emergency}")
                self.graph.receive_emergency(emergency, now)
                self.mission_started = True

            # Get task
            if self.mission_started and now - last_graph >= 0.5:
                commands = self.graph.step(now)
                cmd = commands.get("alpha")
                if cmd and cmd != self.last_commands.get("alpha"):
                    action = cmd.get("action")
                    if action == "pick" and self.nav_phase == "idle":
                        aisle = cmd.get("aisle", "A")
                        current_item = cmd.get("item", "?")
                        self.nav_aisle = aisle
                        self.nav_waypoints = plan_pick_path([x, y], aisle)
                        self.nav_phase = "navigating_to_shelf"
                        log(f"[Task] PICK {current_item} aisle {aisle}")
                        self.last_commands["alpha"] = cmd
                last_graph = now

            # ── State machine ──

            if self.nav_phase == "navigating_to_shelf":
                arrived = self.navigate(x, y, yaw)
                if arrived:
                    log(f"[Shelf] Arrived aisle {self.nav_aisle} at ({x:.2f},{y:.2f})")
                    self.nav_phase = "picking"
                    pick_time = now

            elif self.nav_phase == "picking":
                self.send_cmd(0.0, 0.0)
                if now - pick_time > 1.5:
                    task = self.graph.state.get("active_tasks", {}).get("alpha")
                    if task and isinstance(task, dict):
                        item = task.get("item", "?")
                        self.carrying = item
                        self.graph.task_picked("alpha", item)

                        # Grab the actual box in the simulation
                        self.grab_box(item)

                        # Plan return
                        aisle = self.nav_aisle or "A"
                        self.nav_waypoints = plan_deliver_path([x, y], aisle)
                        self.nav_phase = "navigating_to_drop"
                        self.last_commands["alpha"] = None
                        log(f"[Pick] GOT {item}! Heading to drop")
                    else:
                        log(f"[Pick] No task, idle")
                        self.nav_phase = "idle"

            elif self.nav_phase == "navigating_to_drop":
                arrived = self.navigate(x, y, yaw)
                if arrived:
                    log(f"[Drop] At drop point ({x:.2f},{y:.2f})")
                    self.nav_phase = "dropping"
                    drop_time = now

            elif self.nav_phase == "dropping":
                self.send_cmd(0.0, 0.0)
                if now - drop_time > 1.5:
                    task = self.graph.state.get("active_tasks", {}).get("alpha")
                    task_id = task.get("id") if task and isinstance(task, dict) else None

                    # Place the box at drop location
                    drop = WAYPOINTS.get("TRUCK_DROP", (0.0, -4.2))
                    self.place_box(drop[0], drop[1])

                    if task_id:
                        self.graph.task_delivered("alpha", task_id)
                    log(f"[Drop] DELIVERED {self.carrying}!")
                    self.carrying = None
                    self.nav_phase = "idle"
                    self.nav_waypoints = []
                    self.last_commands["alpha"] = None

            elif self.nav_phase == "idle":
                self.send_cmd(0.0, 0.0)


if __name__ == "__main__":
    sup = FireBotSupervisor()
    sup.run()
