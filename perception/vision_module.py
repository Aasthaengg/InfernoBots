#!/usr/bin/env python3
"""
Vision Module for Webots Robot

Integrates Webots camera with VLM for object detection.
Handles frame capture, preprocessing, and VLM queries.
"""

import numpy as np
import sys
import os

# Add parent path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from perception.vlm_moondream import get_vlm, MoondreamVLM, MockVLM
from typing import Dict, Any, Optional, List


class VisionModule:
    """
    Robot vision system with VLM integration.
    
    Usage:
        vision = VisionModule(robot, use_mock=False)
        result = vision.scan_shelf()
        if result.get("target_visible"):
            location = result.get("location")
    """
    
    def __init__(self, robot, camera_name: str = "camera", use_mock: bool = False):
        """
        Initialize vision module.
        
        Args:
            robot: Webots Robot instance
            camera_name: Name of camera device
            use_mock: If True, use MockVLM instead of real Ollama
        """
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        
        # Initialize camera
        try:
            self.camera = robot.getDevice(camera_name)
            self.camera.enable(self.timestep)
            self.width = self.camera.getWidth()
            self.height = self.camera.getHeight()
            print(f"[Vision] Camera initialized: {self.width}x{self.height}")
        except Exception as e:
            print(f"[Vision] ⚠️ Camera error: {e}")
            self.camera = None
            self.width = 640
            self.height = 480
        
        # Initialize VLM
        self.vlm = get_vlm(use_mock=use_mock)
        
        # Cache for recent detections
        self._last_scan = None
        self._scan_timestamp = 0
    
    def get_frame(self) -> np.ndarray:
        """
        Capture frame from camera.
        
        Returns:
            OpenCV-compatible BGR image (numpy array)
        """
        if self.camera is None:
            # Return blank frame if no camera
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Get raw image data
        image_data = self.camera.getImage()
        
        if image_data is None:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Convert to numpy array (Webots returns BGRA)
        frame = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
        
        # Convert BGRA to BGR
        return frame[:, :, :3].copy()
    
    def save_frame(self, filename: str = "/tmp/robot_view.jpg"):
        """Save current frame to file."""
        try:
            import cv2
            frame = self.get_frame()
            cv2.imwrite(filename, frame)
            return filename
        except ImportError:
            # Use camera's built-in save if cv2 unavailable
            if self.camera:
                self.camera.saveImage(filename, 80)
                return filename
        return None
    
    def scan_shelf(self, target_item: str = None) -> Dict[str, Any]:
        """
        Scan shelf with VLM.
        
        Args:
            target_item: Optional specific item to look for
        
        Returns:
            {
                "objects": [...],
                "target_visible": bool,
                "obstacle_detected": bool,
                "confidence": float
            }
        """
        frame = self.get_frame()
        
        if target_item:
            result = self.vlm.verify_object(frame, target_item)
            # Normalize response format
            return {
                "objects": [target_item] if result.get("found") else [],
                "target_visible": result.get("found", False),
                "target_location": result.get("location"),
                "obstacle_detected": False,
                "confidence": result.get("confidence", 0.0),
                "_inference_time_ms": result.get("_inference_time_ms", 0)
            }
        else:
            result = self.vlm.analyze(frame, prompt_type="shelf_scan")
            self._last_scan = result
            return result
    
    def check_path(self) -> Dict[str, Any]:
        """
        Check if path ahead is clear.
        
        Returns:
            {
                "blocked": bool,
                "obstacle_type": str,
                "clearance_direction": str
            }
        """
        frame = self.get_frame()
        return self.vlm.check_obstacles(frame)
    
    def verify_item(self, item_name: str) -> bool:
        """
        Quick check if specific item is visible.
        
        Args:
            item_name: Name of item to find
            
        Returns:
            True if item found with >50% confidence
        """
        result = self.scan_shelf(target_item=item_name)
        return result.get("target_visible", False) and result.get("confidence", 0) > 0.5
    
    def get_item_location(self, item_name: str) -> Optional[str]:
        """
        Get location of item in frame.
        
        Returns:
            "left", "center", or "right", or None if not found
        """
        result = self.scan_shelf(target_item=item_name)
        if result.get("target_visible"):
            return result.get("target_location")
        return None


class VisionDebugger:
    """
    Debugging tools for vision system.
    """
    
    @staticmethod
    def annotate_frame(frame: np.ndarray, detections: Dict) -> np.ndarray:
        """Add detection annotations to frame."""
        try:
            import cv2
            annotated = frame.copy()
            
            # Add text overlays
            y_offset = 30
            for key, value in detections.items():
                if key.startswith("_"):
                    continue
                text = f"{key}: {value}"
                cv2.putText(annotated, text, (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_offset += 25
            
            return annotated
        except ImportError:
            return frame
    
    @staticmethod
    def log_detection(robot_name: str, result: Dict):
        """Log detection result."""
        print(f"[{robot_name}] Vision: objects={result.get('objects', [])}, "
              f"target={result.get('target_visible', False)}, "
              f"obstacle={result.get('obstacle_detected', False)}, "
              f"conf={result.get('confidence', 0):.2f}, "
              f"time={result.get('_inference_time_ms', 0)}ms")


# ═══════════════════════════════════════════════════════════
# STANDALONE TEST
# ═══════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("VisionModule test (standalone, no Webots)")
    
    # Create mock robot
    class MockRobot:
        def getBasicTimeStep(self):
            return 32
        def getDevice(self, name):
            return None
    
    vision = VisionModule(MockRobot(), use_mock=True)
    
    print("\n1. Shelf scan:")
    result = vision.scan_shelf()
    VisionDebugger.log_detection("TEST", result)
    
    print("\n2. Verify item:")
    found = vision.verify_item("fire_extinguisher")
    print(f"   fire_extinguisher found: {found}")
    
    print("\n3. Check path:")
    path = vision.check_path()
    print(f"   blocked: {path.get('blocked')}, type: {path.get('obstacle_type')}")
