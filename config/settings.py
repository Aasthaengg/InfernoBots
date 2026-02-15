#!/usr/bin/env python3
"""
FireBot Configuration Settings

Central configuration for the multi-robot VLM system.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class RobotConfig:
    """Configuration for a single robot."""
    name: str
    start_position: Tuple[float, float]
    color: str  # For visual identification


@dataclass  
class AisleConfig:
    """Configuration for a warehouse aisle."""
    name: str
    y_position: float
    color: Tuple[float, float, float]  # RGB
    items: List[str]


# ═══════════════════════════════════════════════════════════
# ROBOT CONFIGURATION
# ═══════════════════════════════════════════════════════════

ROBOTS = {
    "alpha": RobotConfig(
        name="youbot_alpha",
        start_position=(-1.0, -2.0),
        color="blue"
    ),
    "beta": RobotConfig(
        name="youbot_beta", 
        start_position=(-2.0, -2.0),
        color="orange"
    )
}


# ═══════════════════════════════════════════════════════════
# WAREHOUSE LAYOUT
# ═══════════════════════════════════════════════════════════

AISLES = {
    "A": AisleConfig(
        name="Fire Suppression",
        y_position=5.5,
        color=(0.95, 0.1, 0.1),  # Red
        items=["ext_co2", "ext_foam", "hose_nozzle", "fire_blanket"]
    ),
    "B": AisleConfig(
        name="Hazmat & Chemical",
        y_position=3.5,
        color=(1.0, 0.5, 0.0),  # Orange
        items=["hazmat_suit", "chemical_neutralizer", "gas_detector", "spill_kit"]
    ),
    "C": AisleConfig(
        name="Medical & Rescue",
        y_position=1.5,
        color=(0.1, 0.8, 0.2),  # Green
        items=["first_aid", "trauma_bag", "defibrillator", "splint_set"]
    ),
    "D": AisleConfig(
        name="Breathing & PPE",
        y_position=-0.5,
        color=(0.2, 0.4, 0.95),  # Blue
        items=["scba_tank", "oxygen_mask", "ppe_bundle", "thermal_camera"]
    )
}

# Item to aisle mapping
ITEM_AISLE: Dict[str, str] = {}
for aisle_id, aisle in AISLES.items():
    for item in aisle.items:
        ITEM_AISLE[item] = aisle_id


# ═══════════════════════════════════════════════════════════
# WAYPOINTS
# ═══════════════════════════════════════════════════════════

WAYPOINTS = {
    # Start positions
    "ALPHA_START": (-1.0, -2.0),
    "BETA_START":  (-2.0, -2.0),
    
    # Spine positions (yellow line at x=-1.5, enter aisle BETWEEN racks)
    "SPINE_A": (-1.5, 4.5),   # aisle between Rack A (y=5.5) and Rack B (y=3.5)
    "SPINE_B": (-1.5, 2.5),   # aisle between Rack B (y=3.5) and Rack C (y=1.5)
    "SPINE_C": (-1.5, 0.5),   # aisle between Rack C (y=1.5) and Rack D (y=-0.5)
    "SPINE_D": (-1.5, -1.5),  # aisle below Rack D (y=-0.5)

    # Shelf approach: walk along aisle line to grid line at x=-2.0
    # Gives 1.35m clearance to rack uprights (x=-3.35) for turning
    "SHELF_A": (-2.0, 4.5),   # grid intersection, between Rack A and B
    "SHELF_B": (-2.0, 2.5),   # grid intersection, between Rack B and C
    "SHELF_C": (-2.0, 0.5),   # grid intersection, between Rack C and D
    "SHELF_D": (-2.0, -1.5),  # grid intersection, below Rack D
    
    # Truck
    "TRUCK_APPROACH": (-1.5, -3.0),
    "TRUCK_DROP":     (-1.5, -4.0),
}

# Navigation constants
SPINE_X = -1.5
SHELF_X = -2.0


# ═══════════════════════════════════════════════════════════
# ARM PRESETS
# ═══════════════════════════════════════════════════════════

ARM_PRESETS = {
    "HOME":     [0.0, -1.13, -2.54, -1.78, 0.0],
    "READY":    [0.0, -0.50, -1.80, -1.30, 0.0],
    "PRE_GRAB": [0.0,  0.10, -1.10, -1.60, 0.0],
    "GRAB":     [0.0,  0.40, -0.70, -1.80, 0.0],
    "LIFT":     [0.0, -0.70, -2.00, -1.00, 0.0],
    "DROP":     [0.0, -0.10, -1.20, -1.50, 0.0],
}

ARM_LIMITS = [
    (-2.949, 2.949),
    (-1.1345, 1.5708),
    (-2.5481, 2.5481),
    (-1.7802, 1.7802),
    (-2.949, 2.949),
]

ARM_SPEEDS = [0.5, 0.8, 0.8, 0.6, 0.3]


# ═══════════════════════════════════════════════════════════
# VLM SETTINGS
# ═══════════════════════════════════════════════════════════

VLM_CONFIG = {
    "model": "moondream",
    "ollama_url": "http://localhost:11434",
    "timeout": 30,
    "use_mock": False,  # Set True for testing without Ollama
}


# ═══════════════════════════════════════════════════════════
# EMERGENCY TYPES
# ═══════════════════════════════════════════════════════════

EMERGENCY_EQUIPMENT = {
    "electrical_fire": ["ext_co2", "scba_tank", "first_aid", "hazmat_suit"],
    "chemical_spill":  ["hazmat_suit", "scba_tank", "first_aid", "ext_co2"],
    "structure_fire":  ["scba_tank", "first_aid", "ext_co2", "hazmat_suit"],
    "wildfire":        ["scba_tank", "hazmat_suit", "first_aid", "ext_co2"],
}


# ═══════════════════════════════════════════════════════════
# TIMING
# ═══════════════════════════════════════════════════════════

TIMESTEP = 32  # Webots timestep (ms)

TIMEOUTS = {
    "navigation": 25.0,
    "arm_motion": 5.0,
    "gripper": 1.0,
    "vlm_inference": 30.0,
}

GPS_BROADCAST_INTERVAL = 0.05  # 50ms
GRAPH_UPDATE_INTERVAL = 0.3   # 300ms
VLM_SCAN_INTERVAL = 2.0       # 2s between shelf scans


# ═══════════════════════════════════════════════════════════
# HELPER FUNCTIONS
# ═══════════════════════════════════════════════════════════

def get_aisle_for_item(item: str) -> str:
    """Get aisle ID for an item."""
    return ITEM_AISLE.get(item, "D")


def get_spine_waypoint(aisle: str) -> Tuple[float, float]:
    """Get spine waypoint for an aisle."""
    return WAYPOINTS.get(f"SPINE_{aisle}", WAYPOINTS["SPINE_D"])


def get_shelf_waypoint(aisle: str) -> Tuple[float, float]:
    """Get shelf waypoint for an aisle."""
    return WAYPOINTS.get(f"SHELF_{aisle}", WAYPOINTS["SHELF_D"])


def get_aisle_y(aisle: str) -> float:
    """Get Y position for an aisle."""
    return AISLES.get(aisle, AISLES["D"]).y_position
