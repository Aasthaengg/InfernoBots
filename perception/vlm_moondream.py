#!/usr/bin/env python3
"""
VLM Integration with Moondream via Ollama (PRODUCTION STRICT)

✅ Real Moondream via Ollama (localhost:11434)
✅ Structured JSON prompts
✅ NO silent mock fallback in production mode

Rules:
- get_vlm(use_mock=False) will HARD-FAIL if Ollama is unavailable
- get_vlm(use_mock=True) is allowed only for local dev/testing
"""

import requests
import base64
import json
import time
from typing import Dict, Any


class MoondreamVLM:
    """
    Moondream 2B Vision-Language Model via Ollama.
    Optimized for warehouse object detection.
    """

    OLLAMA_URL = "http://localhost:11434/api/generate"

    PROMPTS = {
        "shelf_scan": """You are a warehouse robot vision system.
Analyze this shelf image and return ONLY valid JSON:
{
  "objects": ["item1", "item2"],
  "target_visible": false,
  "obstacle_detected": false,
  "confidence": 0.0
}

Rules:
- List ALL visible objects on the shelf
- Set target_visible=true if you see firefighting equipment
- Set obstacle_detected=true if path is blocked
- confidence is 0.0-1.0""",

        "object_verify": """You are verifying if a specific object exists.
Target: {target}

Return ONLY valid JSON:
{{
  "found": true/false,
  "location": "left/center/right",
  "confidence": 0.0
}}""",

        "obstacle_check": """Analyze for obstacles blocking robot path.
Return ONLY valid JSON:
{
  "blocked": true/false,
  "obstacle_type": "none/box/person/unknown",
  "clearance_direction": "left/right/wait"
}"""
    }

    def __init__(self, model: str = "moondream", timeout: int = 30):
        self.model = model
        self.timeout = timeout
        self._check_ollama_strict()

    def _check_ollama_strict(self) -> None:
        """Verify Ollama is running and moondream is available. Raise if not."""
        try:
            response = requests.get("http://localhost:11434/api/tags", timeout=5)
            response.raise_for_status()
            models = [m.get("name", "") for m in response.json().get("models", [])]
            if not any("moondream" in m for m in models):
                raise RuntimeError("Moondream model not found. Run: ollama pull moondream")
            print("[VLM] ✓ Moondream ready")
        except requests.exceptions.ConnectionError as e:
            raise RuntimeError("Ollama not running. Start with: ollama serve") from e
        except Exception as e:
            raise RuntimeError(f"Ollama check failed: {e}") from e

    def encode_image(self, frame) -> str:
        import cv2
        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(buffer).decode("utf-8")

    def analyze(self, frame, prompt: str = None, prompt_type: str = "shelf_scan") -> Dict[str, Any]:
        if prompt is None:
            prompt = self.PROMPTS.get(prompt_type, self.PROMPTS["shelf_scan"])

        image_b64 = self.encode_image(frame)

        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [image_b64],
            "stream": False,
            "options": {
                "temperature": 0.1,
                "num_predict": 256
            }
        }

        t0 = time.time()
        response = requests.post(self.OLLAMA_URL, json=payload, timeout=self.timeout)
        elapsed = time.time() - t0

        if response.status_code != 200:
            raise RuntimeError(f"Ollama returned HTTP {response.status_code}: {response.text[:200]}")

        result_text = response.json().get("response", "")
        parsed = self._parse_json(result_text)
        parsed["_inference_time_ms"] = int(elapsed * 1000)
        return parsed

    def _parse_json(self, text: str) -> Dict[str, Any]:
        try:
            return json.loads(text)
        except Exception:
            pass

        import re
        json_match = re.search(r'\{.*\}', text, re.DOTALL)
        if json_match:
            return json.loads(json_match.group())

        raise ValueError("VLM returned non-JSON output (strict mode).")

    def verify_object(self, frame, target_object: str) -> Dict[str, Any]:
        prompt = self.PROMPTS["object_verify"].format(target=target_object)
        return self.analyze(frame, prompt=prompt)

    def check_obstacles(self, frame) -> Dict[str, Any]:
        return self.analyze(frame, prompt_type="obstacle_check")


class MockVLM:
    """
    Mock VLM for local testing ONLY.
    Do not use in production.
    """

    SHELF_ITEMS = {
        "A": ["fire_extinguisher", "co2_canister", "hose_nozzle"],
        "B": ["hazmat_suit", "chemical_neutralizer", "gas_detector"],
        "C": ["first_aid_kit", "trauma_bag", "defibrillator"],
        "D": ["scba_tank", "oxygen_mask", "ppe_bundle"],
    }

    def __init__(self):
        print("[VLM] ⚠️ Using MockVLM (testing only)")

    def analyze(self, frame, prompt: str = None, prompt_type: str = "shelf_scan") -> Dict[str, Any]:
        import random
        time.sleep(0.05)
        aisle = random.choice(list(self.SHELF_ITEMS.keys()))
        items = self.SHELF_ITEMS[aisle]
        return {
            "objects": items,
            "target_visible": random.random() > 0.2,
            "obstacle_detected": random.random() > 0.9,
            "confidence": round(random.uniform(0.7, 0.95), 2),
            "_inference_time_ms": random.randint(30, 80),
            "_mock": True
        }

    def verify_object(self, frame, target_object: str) -> Dict[str, Any]:
        import random
        return {
            "found": random.random() > 0.3,
            "location": random.choice(["left", "center", "right"]),
            "confidence": round(random.uniform(0.6, 0.95), 2),
            "_mock": True
        }

    def check_obstacles(self, frame) -> Dict[str, Any]:
        import random
        blocked = random.random() > 0.85
        return {
            "blocked": blocked,
            "obstacle_type": "box" if blocked else "none",
            "clearance_direction": "left" if blocked else "none",
            "_mock": True
        }


def get_vlm(use_mock: bool = False):
    """
    Factory function.

    Production:
      get_vlm(use_mock=False) MUST succeed with Ollama + moondream available.
      Otherwise raise RuntimeError (no silent mock fallback).

    Testing:
      get_vlm(use_mock=True) returns MockVLM.
    """
    if use_mock:
        return MockVLM()

    # Strict production: hard-fail if unavailable
    return MoondreamVLM()
