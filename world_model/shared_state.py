#!/usr/bin/env python3
"""
Shared World State for Multi-Robot Coordination

Centralized state that all agents read/write.
Used by LangGraph for coordination decisions.
"""

from typing import TypedDict, List, Dict, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import json
import time


class Phase(Enum):
    """Mission phases."""
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    COORDINATING = "coordinating"
    COMPLETE = "complete"
    FAILED = "failed"


class RobotState(Enum):
    """Robot operational states."""
    IDLE = "idle"
    MOVING_TO_SHELF = "moving_to_shelf"
    SCANNING = "scanning"
    PICKING = "picking"
    MOVING_TO_TRUCK = "moving_to_truck"
    DROPPING = "dropping"
    WAITING = "waiting"
    FAILED = "failed"


@dataclass
class RobotStatus:
    """Status of a single robot."""
    name: str
    position: List[float] = field(default_factory=lambda: [0.0, 0.0])
    yaw: float = 0.0
    state: str = "idle"
    current_task: Optional[str] = None
    carrying: Optional[str] = None
    path_blocked: bool = False
    last_vlm_result: Optional[Dict] = None
    tasks_completed: int = 0
    last_update: float = 0.0
    
    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "position": self.position,
            "yaw": self.yaw,
            "state": self.state,
            "current_task": self.current_task,
            "carrying": self.carrying,
            "path_blocked": self.path_blocked,
            "tasks_completed": self.tasks_completed,
            "last_update": self.last_update,
        }
    
    @staticmethod
    def from_dict(data: Dict) -> 'RobotStatus':
        return RobotStatus(
            name=data.get("name", "unknown"),
            position=data.get("position", [0.0, 0.0]),
            yaw=data.get("yaw", 0.0),
            state=data.get("state", "idle"),
            current_task=data.get("current_task"),
            carrying=data.get("carrying"),
            path_blocked=data.get("path_blocked", False),
            tasks_completed=data.get("tasks_completed", 0),
            last_update=data.get("last_update", 0.0),
        )


@dataclass
class Task:
    """A task to be executed by a robot."""
    id: str
    item: str
    aisle: str
    assigned_to: Optional[str] = None
    status: str = "pending"  # pending, active, complete, failed
    priority: int = 1
    vlm_verified: bool = False
    
    def to_dict(self) -> Dict:
        return {
            "id": self.id,
            "item": self.item,
            "aisle": self.aisle,
            "assigned_to": self.assigned_to,
            "status": self.status,
            "priority": self.priority,
            "vlm_verified": self.vlm_verified,
        }


class WorldState(TypedDict):
    """
    Complete world state for LangGraph coordination.
    
    This is the shared state that flows through all agents.
    """
    # Mission
    phase: str
    emergency_type: str
    mission_id: str
    
    # Robots
    alpha: Dict
    beta: Dict
    
    # Tasks
    task_queue: List[Dict]
    active_tasks: Dict[str, Optional[Dict]]  # robot_name -> task
    completed_tasks: List[str]
    failed_tasks: List[str]
    
    # VLM Detection Results
    vlm_detections: Dict[str, Dict]  # aisle -> detection result
    
    # Coordination
    spine_occupied_by: Optional[str]
    collision_alert: bool
    replan_needed: bool
    
    # Metrics
    start_time: float
    items_delivered: int
    total_items: int


def create_initial_state() -> WorldState:
    """Create fresh world state."""
    return {
        "phase": Phase.IDLE.value,
        "emergency_type": "",
        "mission_id": "",
        
        "alpha": RobotStatus(
            name="youbot_alpha",
            position=[-1.0, -2.0],
        ).to_dict(),
        
        "beta": RobotStatus(
            name="youbot_beta",
            position=[-2.0, -2.0],
        ).to_dict(),
        
        "task_queue": [],
        "active_tasks": {"alpha": None, "beta": None},
        "completed_tasks": [],
        "failed_tasks": [],
        
        "vlm_detections": {},
        
        "spine_occupied_by": None,
        "collision_alert": False,
        "replan_needed": False,
        
        "start_time": 0.0,
        "items_delivered": 0,
        "total_items": 0,
    }


class StateManager:
    """
    Manages world state with thread-safe operations.
    Can be extended to use Redis for distributed state.
    """
    
    def __init__(self):
        self.state = create_initial_state()
        self._lock = False  # Simple lock for now
    
    def get_state(self) -> WorldState:
        """Get current state."""
        return self.state.copy()
    
    def update_robot(self, robot_name: str, updates: Dict):
        """Update robot status."""
        if robot_name in ["alpha", "beta"]:
            self.state[robot_name].update(updates)
            self.state[robot_name]["last_update"] = time.time()
    
    def add_vlm_detection(self, aisle: str, detection: Dict):
        """Record VLM detection result."""
        self.state["vlm_detections"][aisle] = {
            **detection,
            "timestamp": time.time()
        }
    
    def get_pending_tasks(self) -> List[Dict]:
        """Get all pending tasks."""
        return [t for t in self.state["task_queue"] if t["status"] == "pending"]
    
    def get_active_task(self, robot_name: str) -> Optional[Dict]:
        """Get active task for robot."""
        return self.state["active_tasks"].get(robot_name)
    
    def complete_task(self, task_id: str):
        """Mark task as complete."""
        for task in self.state["task_queue"]:
            if task["id"] == task_id:
                task["status"] = "complete"
                self.state["completed_tasks"].append(task_id)
                self.state["items_delivered"] += 1
                break
    
    def fail_task(self, task_id: str):
        """Mark task as failed."""
        for task in self.state["task_queue"]:
            if task["id"] == task_id:
                task["status"] = "failed"
                self.state["failed_tasks"].append(task_id)
                break
    
    def to_json(self) -> str:
        """Serialize state to JSON."""
        return json.dumps(self.state, indent=2, default=str)
    
    def summary(self) -> str:
        """Get brief state summary."""
        return (
            f"Phase: {self.state['phase']} | "
            f"Delivered: {self.state['items_delivered']}/{self.state['total_items']} | "
            f"Alpha: {self.state['alpha']['state']} | "
            f"Beta: {self.state['beta']['state']}"
        )


# Global state manager instance
_state_manager: Optional[StateManager] = None


def get_state_manager() -> StateManager:
    """Get or create global state manager."""
    global _state_manager
    if _state_manager is None:
        _state_manager = StateManager()
    return _state_manager
