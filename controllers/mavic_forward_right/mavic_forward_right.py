"""Mavic 2 Pro Controller: Absolute GPS navigation with encapsulated control."""

from controller import Robot
import sys
import numpy as np


def clamp(value, value_min, value_max):
    """Clamp value between min and max."""
    return min(max(value, value_min), value_max)


def normalize_angle(angle):
    """Normalize angle to ]-pi;pi]"""
    angle = (angle + 2 * np.pi) % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle


class Mavic(Robot):
    """Mavic 2 Pro drone controller with GPS navigation."""

    # Flight control constants (empirically tuned)
    K_VERTICAL_THRUST = 68.5  # Base thrust to hover
    K_VERTICAL_OFFSET = 0.6   # Vertical stabilization offset
    K_VERTICAL_P = 3.0        # Altitude PID constant
    K_ROLL_P = 50.0           # Roll PID constant
    K_PITCH_P = 30.0          # Pitch PID constant

    # Movement parameters
    MAX_YAW_DISTURBANCE = 0.4      # Max turn rate
    MAX_PITCH_DISTURBANCE = -1.0   # Max forward/backward speed
    # TODO - control algorithm for better precision and stability
    TARGET_PRECISION = 0.5         # Target reach precision in meters. Lower than this is not converging.

    def __init__(self):
        """Initialize Mavic 2 Pro with all sensors and motors."""
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        # Initialize sensors
        self._init_sensors()

        # Initialize motors
        self._init_motors()

        # State variables. Initialize to zeros.
        self.gps_position = [0, 0, 0]  # [x, y, altitude]
        self.orientation = [0, 0, 0]    # [roll, pitch, yaw]
        self.angular_velocity = [0, 0, 0]  # [roll_vel, pitch_vel, yaw_vel]

        # Navigation
        self.target_gps = None
        self.target_altitude = 0
        self.target_yaw = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.mission_complete = False

        # Control outputs
        self.roll_disturbance = 0
        self.pitch_disturbance = 0
        self.yaw_disturbance = 0

    def _init_sensors(self):
        """Initialize and enable all sensors."""
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)

        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

    def _init_motors(self):
        """Initialize all motors."""
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")

        # Set camera angle, TODO - make this adjustable for better vision-based navigation
        self.camera_pitch_motor.setPosition(0.7)

        # Set motors to velocity control mode
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

    def update_sensors(self):
        """Read all sensor data and update state."""
        # Get orientation from IMU
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        self.orientation = [roll, pitch, yaw]

        # Get GPS position
        x, y, altitude = self.gps.getValues()
        self.gps_position = [x, y, altitude]

        # Get angular velocities
        roll_vel, pitch_vel, yaw_vel = self.gyro.getValues()
        self.angular_velocity = [roll_vel, pitch_vel, yaw_vel]

    def set_target_altitude(self, altitude):
        """Set target altitude in meters."""
        self.target_altitude = altitude

    def set_target_gps(self, x, y, z=None, yaw=None):
        """
        Set target GPS position (absolute coordinates).

        Args:
            x: Target X coordinate
            y: Target Y coordinate
            z: Target altitude (optional)
            yaw: Target yaw angle in radians (optional)
        """
        self.target_gps = [x, y]
        if z is not None:
            self.target_altitude = z
        if yaw is not None:
            self.target_yaw = yaw

    def set_waypoints(self, waypoints):
        """
        Set list of GPS waypoints to follow.

        Args:
            waypoints: List of [x, y, z, yaw] coordinates in absolute GPS terms
                      z = altitude in meters
                      yaw = heading angle in radians
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        if waypoints:
            wp = waypoints[0]
            self.target_gps = [wp[0], wp[1]]
            if len(wp) > 2:
                self.target_altitude = wp[2]
            if len(wp) > 3:
                self.target_yaw = wp[3]

    def calculate_distance_to_target(self):
        """Calculate Euclidean distance to current GPS target."""
        if self.target_gps is None:
            return 0

        dx = self.target_gps[0] - self.gps_position[0]
        dy = self.target_gps[1] - self.gps_position[1]
        dz = self.target_altitude - self.gps_position[2]
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def calculate_angle_to_target(self):
        """Calculate angle to current GPS target."""
        if self.target_gps is None:
            return 0

        dx = self.target_gps[0] - self.gps_position[0]
        dy = self.target_gps[1] - self.gps_position[1]
        target_angle = np.arctan2(dy, dx)

        # Calculate angle difference from current heading
        angle_diff = normalize_angle(target_angle - self.orientation[2])
        return angle_diff

    def is_target_reached(self):
        """Check if current GPS target is reached."""
        if self.target_gps is None:
            return True

        distance = self.calculate_distance_to_target()
        return distance < self.TARGET_PRECISION

    def advance_to_next_waypoint(self):
        """Move to next waypoint in the list."""
        if not self.waypoints:
            return False

        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            # All waypoints reached
            self.mission_complete = True
            return False

        wp = self.waypoints[self.current_waypoint_index]
        self.target_gps = [wp[0], wp[1]]
        if len(wp) > 2:
            self.target_altitude = wp[2]
        if len(wp) > 3:
            self.target_yaw = wp[3]
        return True

    def calculate_navigation_disturbances(self):
        """
        Calculate yaw and pitch disturbances to navigate to GPS target.

        Returns:
            tuple: (yaw_disturbance, pitch_disturbance)
        """
        if self.target_gps is None:
            return 0, 0

        # Calculate angle to target
        angle_to_target = self.calculate_angle_to_target()

        # Calculate yaw disturbance (proportional to angle)
        yaw_dist = self.MAX_YAW_DISTURBANCE * angle_to_target / (2 * np.pi)

        # Core navigation disturbance algorithm - logarithmic decay based on angle to target
        # This would make the drone slow down as it gets closer to the target angle, but still allow for some disturbance when the angle is small.
        pitch_dist = clamp(
            np.log10(abs(angle_to_target) + 0.01),
            self.MAX_PITCH_DISTURBANCE,
            0.1
        )
        # if abs(yaw_dist) >= np.radians(5):
        #     pitch_dist = 0 

        return yaw_dist, pitch_dist

    def stop_motors(self):
        """Stop all motors by setting velocity to zero."""
        slow_down_rpm = 0  # Very low RPM to effectively slower hower down
        self.front_left_motor.setVelocity(slow_down_rpm)
        self.front_right_motor.setVelocity(-slow_down_rpm)
        self.rear_left_motor.setVelocity(-slow_down_rpm)
        self.rear_right_motor.setVelocity(slow_down_rpm)

    def update_motors(self):
        """Calculate and apply motor velocities based on PID control."""
        roll, pitch, yaw = self.orientation
        roll_vel, pitch_vel, yaw_vel = self.angular_velocity
        altitude = self.gps_position[2]

        # PID control inputs
        roll_input = (self.K_ROLL_P * clamp(roll, -1, 1) +
                     roll_vel + self.roll_disturbance)

        pitch_input = (self.K_PITCH_P * clamp(pitch, -1, 1) +
                      pitch_vel + self.pitch_disturbance)

        yaw_input = self.yaw_disturbance

        # Vertical control
        altitude_error = self.target_altitude - altitude + self.K_VERTICAL_OFFSET
        clamped_altitude_error = clamp(altitude_error, -1, 1)
        vertical_input = self.K_VERTICAL_P * pow(clamped_altitude_error, 3.0)

        # Calculate motor velocities
        front_left = (self.K_VERTICAL_THRUST + vertical_input -
                     yaw_input + pitch_input - roll_input)
        front_right = (self.K_VERTICAL_THRUST + vertical_input +
                      yaw_input + pitch_input + roll_input)
        rear_left = (self.K_VERTICAL_THRUST + vertical_input +
                    yaw_input - pitch_input - roll_input)
        rear_right = (self.K_VERTICAL_THRUST + vertical_input -
                     yaw_input - pitch_input + roll_input)

        # Apply to motors (note: front-right and rear-left are inverted)
        self.front_left_motor.setVelocity(front_left)
        self.front_right_motor.setVelocity(-front_right)
        self.rear_left_motor.setVelocity(-rear_left)
        self.rear_right_motor.setVelocity(rear_right)

    def goto_gps(self, x, y, z=None, yaw=None, verbose=False):
        """
        Navigate to absolute GPS coordinates.

        Args:
            x: Target X coordinate (meters)
            y: Target Y coordinate (meters)
            z: Target altitude (meters), optional
            yaw: Target heading angle (radians), optional
            verbose: Print navigation info
        """
        self.set_target_gps(x, y, z, yaw)

        if verbose:
            yaw_str = f", yaw: {np.degrees(yaw):.1f}°" if yaw is not None else ""
            print(f"Navigating to GPS: [{x:.2f}, {y:.2f}] at altitude {self.target_altitude:.2f}m{yaw_str}")

    def navigate_waypoints(self, verbose=False):
        """
        Navigate through waypoints, returns True if still navigating.

        Args:
            verbose: Print navigation progress

        Returns:
            bool: True if still navigating, False if all waypoints reached
        """
        if not self.waypoints or self.mission_complete:
            return False

        # Check if current waypoint reached
        if self.is_target_reached():
            if verbose:
                wp = self.waypoints[self.current_waypoint_index]
                print(f"✓ Waypoint {self.current_waypoint_index + 1} [{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}m, {np.degrees(wp[3]) if len(wp) > 3 else 0:.1f}°] reached!")

            # Try to advance to next waypoint
            if not self.advance_to_next_waypoint():
                if verbose:
                    print("=" * 60)
                    print("✓✓✓ MISSION COMPLETE - All waypoints reached! ✓✓✓")
                    print("=" * 60)
                return False

            if verbose:
                next_wp = self.waypoints[self.current_waypoint_index]
                print(f"→ Next waypoint {self.current_waypoint_index + 1}: [{next_wp[0]:.1f}, {next_wp[1]:.1f}, {next_wp[2]:.1f}m, {np.degrees(next_wp[3]) if len(next_wp) > 3 else 0:.1f}°]")

        # Calculate navigation disturbances
        self.yaw_disturbance, self.pitch_disturbance = self.calculate_navigation_disturbances()

        if verbose:
            distance = self.calculate_distance_to_target()
            angle = self.calculate_angle_to_target()
            print(f"  Distance: {distance:.2f}m, Angle: {np.degrees(angle):.1f}°")

        return True

    def run(self):
        """Main control loop."""
        # Mission configuration - ABSOLUTE GPS COORDINATES [x, y, z, yaw]
        # x, y: GPS coordinates (meters)
        # z: altitude (meters)
        # yaw: heading angle (radians, 0=East, π/2=North, π=West, -π/2=South)
        waypoints = [
            [0, 0, 10, 0],           # Start: origin, 10m altitude, facing East
            [10, 0, 10, 0],         # Waypoint 1: 100m East, 10m altitude
            [10, -10, 10, -np.pi/2] # Waypoint 2: 100m South, 10m altitude, facing South
        ]

        self.set_waypoints(waypoints)

        print("=" * 60)
        print("Mavic 2 Pro - Absolute GPS Navigation")
        print("=" * 60)
        print("Mission waypoints (absolute GPS coordinates):")
        for i, wp in enumerate(waypoints):
            yaw_deg = np.degrees(wp[3]) if len(wp) > 3 else 0
            print(f"  {i+1}. GPS [{wp[0]:6.1f}, {wp[1]:6.1f}] Alt: {wp[2]:.1f}m Yaw: {yaw_deg:6.1f}°")
        print("=" * 60)

        update_timer = self.getTime()
        initial_altitude_reached = False

        while self.step(self.time_step) != -1:
            # Update sensor readings
            self.update_sensors()

            # Wait until we reach initial altitude before starting navigation
            if not initial_altitude_reached:
                if self.gps_position[2] > self.target_altitude - 1:
                    initial_altitude_reached = True
                    print(f"✓ Initial altitude {self.target_altitude:.1f}m reached. Starting navigation...")
            else:
                # Update navigation every 100ms
                if self.getTime() - update_timer > 0.1:
                    still_navigating = self.navigate_waypoints(verbose=True)
                    update_timer = self.getTime()

                    # Exit when mission complete
                    if not still_navigating:
                        print(f"\nMission complete! Stopping motors and exiting in 2 seconds...")

                        # Stop all motors
                        self.stop_motors()

                        # Wait 2 seconds with motors stopped
                        end_time = self.getTime() + 2
                        while self.step(self.time_step) != -1 and self.getTime() < end_time:
                            pass

                        print("✓ Motors stopped. Exiting simulation.")
                        break

            # Update motor velocities
            self.update_motors()


# Main execution
if __name__ == "__main__":
    robot = Mavic()
    robot.run()
