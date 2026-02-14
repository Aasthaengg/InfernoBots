"""MissionBrain: high-level decision layer (LLM/VLM ready) for Mavic.

Design goals:
- NEVER touch low-level motors or gains.
- Only suggest high-level actions: move(target_xy) or hover.
- Non-blocking: returns quickly; heavy ops are rate-limited.
- If LLM/VLM are unavailable, it no-ops (falls back to controller defaults).

Hook usage (see mavic_fire_multi.py):
    brain = MissionBrain(controller)
    cmd = brain.tick(camera, x, y, alt)
    if cmd: apply high-level action.

LLM/VLM integration points are stubbed; wire them by filling `_run_vlm` and
`_run_llm`. Keep them fast or move to async/thread if needed.
"""

import time
import json
from typing import Optional, Dict, Any


class MissionBrain:
    def __init__(self, controller, llm_enabled=False, vlm_enabled=False):
        self.ctrl = controller
        self.llm_enabled = llm_enabled
        self.vlm_enabled = vlm_enabled

        # timers to throttle expensive calls
        self.last_vlm = 0.0
        self.last_llm = 0.0
        self.vlm_period = 1.0   # seconds
        self.llm_period = 2.0   # seconds

        # state shared with LLM
        self.mission_state = {
            "phase": "patrolling",
            "detected_fires": [],
            "visited_zones": [],
            "current_target": None,
        }

    # -------- Public entry --------
    def tick(self, camera, x, y, alt) -> Optional[Dict[str, Any]]:
        """Called every control step. Returns a dict action or None.
        action schema: {"action": "move"|"hover", "target": (x,y)|None}
        """
        now = time.time()

        # Optionally run VLM
        vlm_out = None
        if self.vlm_enabled and now - self.last_vlm > self.vlm_period:
            vlm_out = self._run_vlm(camera)
            self.last_vlm = now

        # Optionally run LLM
        if self.llm_enabled and now - self.last_llm > self.llm_period:
            decision = self._run_llm(vlm_out, x, y, alt)
            self.last_llm = now
            return decision

        # Default: no-op (lets controller do baseline behavior)
        return None

    # -------- Stubs to be filled with real models --------
    def _run_vlm(self, camera):
        # placeholder: return perception dict
        return {
            "fire_detected": False,
            "severity": None,
            "fire_pixels": 0,
        }

    def _run_llm(self, vlm_out, x, y, alt):
        # If VLM says fire, set a move target to current position (hover)
        if vlm_out and vlm_out.get("fire_detected"):
            return {"action": "hover", "target": None}
        return None

    # -------- Helpers for future robust decisions --------
    def _deduplicate_fire(self, x, y, radius=3.0):
        for f in self.mission_state["detected_fires"]:
            fx, fy = f["gps"]
            if (fx - x) ** 2 + (fy - y) ** 2 < radius ** 2:
                return True
        return False

    def record_fire(self, x, y, severity="unknown"):
        if not self._deduplicate_fire(x, y):
            self.mission_state["detected_fires"].append({
                "gps": (x, y),
                "severity": severity,
                "ts": time.time(),
            })
