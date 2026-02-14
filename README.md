# Mavic Drone Autonomous Mission

A Webots simulation of a Mavic drone executing an autonomous L-shaped flight pattern.

## Prerequisites

- **Webots R2023b** (compatible with Ubuntu 20.04)
- **Python 3.7+** with NumPy
- **Ubuntu 20.04** or compatible Linux distribution

### Installing Webots

If you haven't installed Webots yet:

```bash
# For Ubuntu 20.04
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb
sudo apt install ./webots_2023b_amd64.deb
```

For other versions, see: https://github.com/cyberbotics/webots/releases

## Quick Start

### 1. Launch the Simulation
```bash
webots worlds/mission.wbt
```

### 2. Run the Simulation
Once Webots opens:
- The simulation will be **running**
- We are running the control as well to follow the waypoints
- Watch the console output for mission progress

## Project Structure

```
hackathon/
├── README.md                              # This file
├── controllers/
│   └── mavic_forward_right/
│       └── mavic_forward_right.py         # Python flight controller
└── worlds/
    └── mission.wbt                        # Webots world file
```

## Mission Overview

The drone performs the following autonomous flight sequence:
1. **Take off** to 10 meters altitude
2. **Fly forward** 10 meters (North)
3. **Turn right** 90 degrees
4. **Fly forward** 10 meters (East)

Total flight distance: **20 meters** in an L-shaped pattern.


## Customizing the Mission

### Change Waypoints

Edit `controllers/mavic_forward_right/mavic_forward_right.py` (line 338):

**Add more waypoints:**
```python
waypoints = [
    [0, 0, 15, 0],
    [12, 0, 15, 0],
    [12, -5, 15, -np.pi/2],
    [0, -5, 15, -np.pi], 
    [0, 0, 15, np.pi/2]       # Return to start
]
```

### Adjust Flight Speed

Modify PID constants:
```python
K_VERTICAL_THRUST = 68.5  # Hover thrust
K_VERTICAL_P = 3.0        # Vertical control
K_ROLL_P = 50.0           # Roll control (higher = faster turns)
K_PITCH_P = 30.0          # Pitch control (higher = faster forward/back)
```

### Change Precision

Edit target precision (line 44):
```python
target_precision = 0.5  # Lower = more precise (slower), higher = less precise (faster)
```



## API Reference

For detailed drone control API documentation, see the official Webots documentation:
- [Webots Robot API](https://cyberbotics.com/doc/reference/robot)
- [Supervisor API](https://cyberbotics.com/doc/reference/supervisor)
- [Python Controller](https://cyberbotics.com/doc/guide/using-python)

## Resources

- [Webots Documentation](https://cyberbotics.com/doc/guide/index)
- [Webots GitHub](https://github.com/cyberbotics/webots)
- [Python Controller Guide](https://cyberbotics.com/doc/guide/controller-programming)

---
