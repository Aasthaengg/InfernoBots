"""Autonomous Mavic 2 Pro controller (no ROS / no LLM).

Behavior
- Take off to a fixed altitude.
- Patrol a simple square route until a fire is visually detected in the RGB camera.
- On detection, record the current GPS as the fire location, fly to it, hover, log it, resume patrol.

Fire detection is purely color based (red/orange dominance).
"""

from controller import Supervisor
import math
import time

from mission_brain import MissionBrain  # lightweight, high-level only

MISSION_PATROL = 0
MISSION_GO_TO_FIRE = 1
MISSION_HOVER = 2


# ------------------ helpers ------------------
def clamp(val, lo, hi):
    return min(max(val, lo), hi)


def normalize_angle(a):
    a = (a + 2 * math.pi) % (2 * math.pi)
    if a > math.pi:
        a -= 2 * math.pi
    return a


class MavicController(Supervisor):
    # PID gains (from Webots DJI example)
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    MAX_YAW_DIST = 0.4
    MAX_PITCH_DIST = -3.0
    TARGET_PRECISION = 1.0

    TARGET_ALT = 12.0  # meters

    def __init__(self):
        super().__init__()
        self.dt = int(self.getBasicTimeStep())

        # Devices
        self.cam = self.getDevice("camera")
        self.cam.enable(self.dt)
        self.imu = self.getDevice("inertial unit"); self.imu.enable(self.dt)
        self.gps = self.getDevice("gps"); self.gps.enable(self.dt)
        self.gyro = self.getDevice("gyro"); self.gyro.enable(self.dt)

        self.front_left = self.getDevice("front left propeller")
        self.front_right = self.getDevice("front right propeller")
        self.rear_left = self.getDevice("rear left propeller")
        self.rear_right = self.getDevice("rear right propeller")
        for m in (self.front_left, self.front_right, self.rear_left, self.rear_right):
            m.setPosition(float('inf'))
            m.setVelocity(1)

        self.cam_pitch = self.getDevice("camera pitch")
        self.cam_pitch.setPosition(1.2)  # look downwards to see fires

        # Mission state
        self.state = MISSION_PATROL
        self.fire_target = None
        self.hover_timer_ms = 0

        # Patrol waypoints (simple square sweep)
        self.patrol_wps = [
            (0, 0),
            (40, 0),
            (40, -40),
            (0, -40),
            (-40, -40),
            (-40, 0),
        ]
        self.patrol_idx = 0

        # Control outputs
        self.yaw_dist = 0
        self.pitch_dist = 0

        # Log file
        self.log_path = "/home/aastha/firebot/InfernoBots/controllers/mavic_fire_multi/fire_log.txt"

        # Initialize target
        self.set_target_xy(*self.patrol_wps[self.patrol_idx])
        # Immediately move toward first leg of patrol (not origin hover)
        self.patrol_next_wp()

        # High-level brain (non-blocking, no low-level control)
        self.brain = MissionBrain(self)

    # -------------- sensing --------------
    def read_state(self):
        self.roll, self.pitch, self.yaw = self.imu.getRollPitchYaw()
        self.x, self.y, self.alt = self.gps.getValues()
        self.roll_vel, self.pitch_vel, self.yaw_vel = self.gyro.getValues()

    # -------------- detection --------------
    def detect_fire(self):
        w, h = self.cam.getWidth(), self.cam.getHeight()
        img = self.cam.getImage()
        fire_pixels = 0
        for x in range(0, w, 4):
            for y in range(0, h, 4):
                r = self.cam.imageGetRed(img, w, x, y)
                g = self.cam.imageGetGreen(img, w, x, y)
                b = self.cam.imageGetBlue(img, w, x, y)
                if r > 200 and g > 100 and b < 80 and r > g + 40:
                    fire_pixels += 1
        return fire_pixels > 50

    # -------------- navigation helpers --------------
    def set_target_xy(self, tx, ty):
        self.target_x = tx
        self.target_y = ty

    def distance_to_target(self):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.TARGET_ALT - self.alt
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def update_disturbances(self):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        ang = normalize_angle(math.atan2(dy, dx) - self.yaw)
        self.yaw_dist = self.MAX_YAW_DIST * ang / (2 * math.pi)
        self.pitch_dist = clamp(math.log10(abs(ang) + 0.01), self.MAX_PITCH_DIST, 0.1)

    # -------------- controller --------------
    def compute_motors(self):
        roll_input = self.K_ROLL_P * clamp(self.roll, -1, 1) + self.roll_vel
        pitch_input = self.K_PITCH_P * clamp(self.pitch, -1, 1) + self.pitch_vel + self.pitch_dist
        yaw_input = self.yaw_dist

        alt_err = clamp(self.TARGET_ALT - self.alt + self.K_VERTICAL_OFFSET, -1, 1)
        vertical = self.K_VERTICAL_P * (alt_err ** 3)

        fl = self.K_VERTICAL_THRUST + vertical - yaw_input + pitch_input - roll_input
        fr = self.K_VERTICAL_THRUST + vertical + yaw_input + pitch_input + roll_input
        rl = self.K_VERTICAL_THRUST + vertical + yaw_input - pitch_input - roll_input
        rr = self.K_VERTICAL_THRUST + vertical - yaw_input - pitch_input + roll_input

        self.front_left.setVelocity(fl)
        self.front_right.setVelocity(-fr)
        self.rear_left.setVelocity(-rl)
        self.rear_right.setVelocity(rr)

    # -------------- logging --------------
    def log_fire(self, x, y):
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(self.log_path, "a") as f:
            f.write(f"{ts} fire @ ({x:.2f}, {y:.2f})\n")
        print(f"[DRONE] Logged fire @ ({x:.2f}, {y:.2f})")

    # -------------- mission logic --------------
    def patrol_next_wp(self):
        self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol_wps)
        tx, ty = self.patrol_wps[self.patrol_idx]
        self.set_target_xy(tx, ty)

    def run(self):
        while self.step(self.dt) != -1:
            self.read_state()

            # -------- High-level brain hook (non-blocking) --------
            brain_cmd = self.brain.tick(self.cam, self.x, self.y, self.alt)
            if brain_cmd:
                act = brain_cmd.get("action")
                tgt = brain_cmd.get("target")
                if act == "move" and tgt:
                    self.set_target_xy(tgt[0], tgt[1])
                    self.state = MISSION_GO_TO_FIRE
                    print(f"[DRONE] Brain set move target {tgt}")
                elif act == "hover":
                    self.state = MISSION_HOVER
                    self.yaw_dist = 0
                    self.pitch_dist = 0
                    self.hover_timer_ms = 0
                    print("[DRONE] Brain requested hover")
                # continue falls back to baseline logic

            # Fire detection always on during patrol
            if self.state == MISSION_PATROL and self.detect_fire():
                self.fire_target = (self.x, self.y)
                self.set_target_xy(self.fire_target[0], self.fire_target[1])
                self.state = MISSION_GO_TO_FIRE
                print("[DRONE] Fire detected. Navigating to target.")

            if self.state == MISSION_PATROL:
                if self.distance_to_target() < 1.5:
                    self.patrol_next_wp()
                self.update_disturbances()

            elif self.state == MISSION_GO_TO_FIRE:
                self.update_disturbances()
                if self.distance_to_target() < 0.8:
                    self.state = MISSION_HOVER
                    self.hover_timer_ms = 0
                    self.yaw_dist = 0
                    self.pitch_dist = 0
                    print("[DRONE] On fire location. Hovering.")

            elif self.state == MISSION_HOVER:
                self.yaw_dist = 0
                self.pitch_dist = 0
                self.hover_timer_ms += self.dt
                if self.hover_timer_ms > 5000:
                    self.log_fire(self.x, self.y)
                    self.state = MISSION_PATROL
                    self.patrol_next_wp()

            # Apply motor commands
            self.compute_motors()


if __name__ == "__main__":
    MavicController().run()
