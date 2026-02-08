# FireBot Autonomous Emergency Response System

Complete overview of the multi-robot fire station emergency response simulation.

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│           Webots Simulation (firestation.wbt)          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │      SUPERVISOR (firebot_supervisor.py)          │  │
│  │  • Queues missions                               │  │
│  │  • Dispatches tasks via Emitter (ch.1)           │  │
│  │  • Receives robot status via Receiver (ch.1)     │  │
│  │  • Monitors progress & logs                      │  │
│  └──────────────────────────────────────────────────┘  │
│                          ↑ ↓                            │
│              Channel 1 (JSON Messages)                  │
│                          ↑ ↓                            │
│  ┌──────────────────────────────────────────────────┐  │
│  │      youBot (youbot_loader.py)                   │  │
│  │  • Receives dispatch commands                    │  │
│  │  • Collects equipment from shelves               │  │
│  │  • Transports to truck platform                  │  │
│  │  • Sends status updates                          │  │
│  │                                                  │  │
│  │  State Machine:                                  │  │
│  │  IDLE → MOVING_TO_SHELF → POSITIONING_ARM →    │  │
│  │  GRASPING → LIFTING → MOVING_TO_TRUCK →        │  │
│  │  DROPPING → RETURNING → IDLE                    │  │
│  └──────────────────────────────────────────────────┘  │
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Fire Station World Objects                      │  │
│  │  • 5 Equipment Shelves (color-coded)             │  │
│  │  • Truck Platform (loading bay)                  │  │
│  │  • Helipad (for future Mavic2Pro)               │  │
│  │  • Storage Area (barrels, pallets)              │  │
│  └──────────────────────────────────────────────────┘  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Component Details

### Supervisor (`firebot_supervisor.py`)

**Role:** Central mission planner and robot coordinator

**Responsibilities:**
- Queue multi-task missions
- Dispatch commands to robots
- Monitor robot status (1Hz updates)
- Provide feedback on mission progress
- Handle task sequencing

**Communication:**
- Sends: `{command: "dispatch", targets: [...]}`
- Receives: `{robot: "youBot", state: "...", position: [...], ...}`

**Current Behavior:**
- Automatically queues equipment collection mission on startup
- Sends single dispatch to youBot with list of 5 shelves
- Logs status every 0.5 seconds
- Continues running through task completion

### youBot Controller (`youbot_loader.py`)

**Role:** Autonomous equipment collection robot

**Architecture:** Finite State Machine (8 states)

```
┌─────────────────┐
│      IDLE       │ ← Waits for dispatch
└────────┬────────┘
         │ (recv dispatch)
         ↓
┌──────────────────────────────┐
│ For each target shelf:        │
│                              │
│ MOVING_TO_SHELF              │
│   ↓                          │
│ POSITIONING_ARM              │
│   ↓                          │
│ GRASPING                     │
│   ↓                          │
│ LIFTING                      │
│   ↓                          │
│ MOVING_TO_TRUCK              │
│   ↓                          │
│ DROPPING                     │
│   ↓                          │
│ RETURNING (to next shelf)    │
│                              │
└──────────────────────────────┘
         │ (all done)
         ↓
      IDLE
```

**Arm Configurations:**
- `home`: [0, -1.57, -2.635, -1.78, 0] - Safe storage
- `grab_height`: [0, -0.8, -2.4, -1.8, 0] - Reach down to shelf
- `reach_forward`: [0, -1.2, -2.2, -1.8, 0] - Safe carrying height

**Navigation:**
- Dead-reckoning odometry from wheel encoders
- Proportional heading control to face targets
- ~3 m/s max wheel velocity
- Waypoint-following with distance-based arrival detection

**Key Features:**
- Position sensor feedback for arm configuration verification
- Gripper force detection (via sensors)
- ~5 second timeout per state with error recovery
- JSON-based status reporting at ~1Hz

### World File (`worlds/firestation.wbt`)

**Scene Graph:**
```
WorldInfo (basicTimeStep=16ms)
├── Viewpoint (follow "YOUBOT")
├── Background (sky)
├── DirectionalLight
├── Floor (14×14m)
├── Walls (5 sections)
│   ├── Back, Left, Right (full perimeter)
│   └── Front Left/Right (bay opening)
├── Zone Markers (visual reference)
│   ├── Equipment Zone
│   ├── Loading Zone
│   └── Helipad Zone + H marker
├── Equipment Shelves (5 tables)
│   └── Each at height 0.25m with 0.5m reach distance
├── Supply Boxes (6 boxes on shelves)
│   ├── Red, Orange, White, Blue, Green boxes
│   └── Brown cardboard box
├── Truck Components
│   ├── Platform (0.35m high)
│   ├── Cabin (red, 1.4m tall)
│   └── Bed (2.6×1.6m)
├── Props
│   ├── Oil Barrels (3 cylinders)
│   └── Wooden Pallets (2 boxes)
├── youBot (Solid placeholder)
│   ├── Receiver (channel 1)
│   └── Emitter (channel 1)
├── Mavic2Pro (Solid placeholder)
└── Supervisor (Robot node)
    ├── Emitter (channel 1)
    ├── Receiver (channel 1)
    └── supervisor=TRUE
```

**Coordinates:**
- Floor: 14×14m at z=0
- Equipment Zone: (-3, 3) with 6×5m area
- Shelves: Y=5 (3 shelves) and Y=3 (2 shelves)
- Loading Bay: (0, -3.5)
- Helipad: (4.5, 3.5)
- Home: (-1, 1)

## Message Flow

### Mission Start Sequence

```
t=0.0s   [Supervisor] Initializes, queues mission
t=0.5s   [Supervisor] Sends dispatch to youBot:
         {
           "command": "dispatch",
           "task": "collect_equipment",
           "targets": ["fire_suppression", "hazmat", "medical", "breathing", "rescue"]
         }

t=0.5s   [youBot] Receives dispatch
         State: IDLE → MOVING_TO_SHELF
         Current shelf: fire_suppression at (-5, 5)

t=1.0s   [youBot] Status update:
         {
           "robot": "youBot",
           "state": "MOVING_TO_SHELF",
           "position": [-1.2, 1.3],
           "target_idx": 0,
           "targets_total": 5
         }

~3s      [youBot] Reaches shelf
         State: POSITIONING_ARM
         Lower arm to grab_height

~5s      [youBot] Arm positioned
         State: GRASPING
         Close gripper

~6s      [youBot] Item grasped
         State: LIFTING
         Raise arm to reach_forward

~7s      [youBot] Moving to truck
         State: MOVING_TO_TRUCK

~10s     [youBot] Reaches truck platform
         State: DROPPING
         Lower arm, open gripper

~12s     [youBot] Item dropped
         State: RETURNING
         Navigate back to home

~15s     [youBot] Back at home
         State: MOVING_TO_SHELF
         Current shelf: hazmat at (-3, 5)
         [Repeat cycle for each shelf]

~80s     [youBot] Mission complete
         All 5 items collected and on truck
         State: IDLE
```

## Sensor & Actuator Requirements

### youBot Devices (from Webots)

**Motors (6 + 4):**
- `wheel1`, `wheel2`, `wheel3`, `wheel4` - Wheel drive
- `arm1` - Base rotation
- `arm2` - Shoulder lift
- `arm3` - Elbow
- `arm4` - Wrist 1
- `arm5` - Wrist 2
- `finger_left`, `finger_right` - Gripper

**Sensors (9):**
- `arm1_sensor` through `arm5_sensor` - Joint positions
- `finger_left_sensor`, `finger_right_sensor` - Gripper feedback
- `wheel1_sensor` through `wheel4_sensor` - Encoders (for odometry)

**Communication:**
- Receiver channel 1 (dispatch commands)
- Emitter channel 1 (status reports)

## Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Simulation Timestep | 16 ms | Webots basicTimeStep |
| Status Update Rate | ~1 Hz | Sent periodically by youBot |
| Arm Movement Time | 2-3 s | Per configuration |
| Navigation Speed | 0.3-1.5 m/s | Proportional to distance |
| State Timeout | 5 s | Before error handling |
| Full Mission (5 items) | ~80 s | Estimated wall-clock time |

## Running the Simulation

### Setup
1. Ensure Webots R2025a is installed
2. Controllers in: `controllers/youbot_loader/` and `controllers/firebot_supervisor/`
3. World file: `worlds/firestation.wbt`

### Execution
```bash
webots worlds/firestation.wbt
# Simulation auto-starts
# Monitor console for status output
# Press spacebar to play/pause
# Check youBot progress in 3D view
```

### Expected Console Output
```
[youBot] Controller initialized
[Supervisor] FireBot system initialized
[Supervisor] Mission queued: Collect all equipment
[Supervisor] Time: 0.5s
  Queue: 0 missions pending
  youBot: MOVING_TO_SHELF
    Position: (-1.50, 1.00)
    Holding: False, Progress: 0/5
[youBot] State: MOVING_TO_SHELF
[Supervisor] Time: 1.0s
  youBot: MOVING_TO_SHELF
    Position: (-2.10, 2.45)
    Holding: False, Progress: 0/5
...
```

## Future Enhancements

### Phase 2: Mavic2Pro Controller
- Aerial reconnaissance of fire area
- Image-based thermal scanning
- Coordinates with youBot

### Phase 3: Real-time Replanning
- Obstacle detection and avoidance
- Dynamic mission rerouting
- Task reordering for efficiency

### Phase 4: Hardware Integration
- ROS interface for real robots
- Hardware sensors and actuators
- Network communication over Ethernet

## Troubleshooting

### youBot Not Moving
- Check receiver is enabled with `enable(timestep)`
- Verify supervisor sends dispatch message
- Check JSON format of dispatch command

### Arm Not Positioning
- Verify arm sensors are enabled
- Check joint angle limits in ARM_CONFIGS
- Add debug logging in `set_arm_config()`

### No Status Updates
- Check emitter is sending to channel 1
- Verify supervisor receiver is listening on channel 1
- Check JSON encoding in `send_status()`

### Simulation Crashes
- May indicate invalid world file syntax
- Check `firestation.wbt` for unclosed braces
- Verify all Solid geometries are properly defined

## References

- Webots Documentation: https://cyberbotics.com/doc/reference/nodes/robot
- youBot KUKA: Standard omnidirectional robot in Webots
- JSON Protocol: Human-readable inter-robot communication
- State Machines: Robust control flow for robotics

---

**System Version:** 1.0
**Last Updated:** 2026-02-08
**Status:** Functional, ready for testing
