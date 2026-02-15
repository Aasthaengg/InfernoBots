#!/usr/bin/env python3
"""
Vision API for FireBot youBot Camera
Integrates Gemini VLM for intelligent shelf analysis
Falls back to mock responses when no API key is set
"""

import json
import base64
import os
import sys


def analyze_shelf(image_path, mission_context, aisle_info, use_mock=None):
    """
    Send shelf image to Gemini VLM for intelligent equipment identification.
    Falls back to mock mode if no API key set.

    Args:
        image_path: Path to shelf image (JPG)
        mission_context: Emergency type (e.g. "electrical_fire", "chemical_spill")
        aisle_info: Dict with aisle name and category
        use_mock: Force mock mode if True, force real if False, auto-detect if None

    Returns:
        Dict with: selected_item, position, reasoning, confidence, size, priority
    """
    if use_mock is None:
        use_mock = not os.environ.get("GEMINI_API_KEY")

    if use_mock:
        return _mock_response(aisle_info)

    return _gemini_response(image_path, mission_context, aisle_info)


def _gemini_response(image_path, mission_context, aisle_info):
    """Real Gemini VLM call with vision understanding"""
    try:
        import google.generativeai as genai

        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            print("[Vision] No GEMINI_API_KEY set, using mock mode")
            return _mock_response(aisle_info)

        genai.configure(api_key=api_key)
        model = genai.GenerativeModel("gemini-2.0-flash")

        # Read image
        with open(image_path, "rb") as f:
            image_data = f.read()

        prompt = f"""You are the vision system of an autonomous fire station robot.

EMERGENCY TYPE: {mission_context}
CURRENT LOCATION: Aisle {aisle_info['name']} — {aisle_info['category']}

You are looking at a fire station equipment shelf from the youBot's ground-level
camera perspective. The shelf is at 0.4m height with equipment boxes of varying sizes.

Based on the emergency type, identify:
1. Which visible item is MOST CRITICAL for this specific emergency
2. Its position relative to others (leftmost / center-left / center / center-right / rightmost)
3. Scientific reasoning (2 sentences max) for why this item is needed
4. Your confidence level (0-1)
5. Size category of the box

Critical priorities by emergency:
- Electrical fires: CO2 (non-conductive), insulation, PPE, thermal imaging
- Chemical spills: Hazmat suits, neutralizers, gas detectors, containment
- Medical: Trauma bags, defibrillators, first aid, airway management
- Fires: SCBA tanks, thermal cameras, hoses, breathing apparatus

Respond ONLY with valid JSON (no markdown, no code blocks):
{{
  "selected_item": "name of equipment",
  "position": "leftmost|center-left|center|center-right|rightmost",
  "reasoning": "2-sentence scientific reason",
  "confidence": 0.85,
  "size": "small|medium|large",
  "priority": "critical|high|medium|low"
}}"""

        response = model.generate_content([
            prompt,
            {"mime_type": "image/jpeg", "data": base64.b64encode(image_data).decode()}
        ])

        text = response.text.strip()

        # Remove markdown code blocks if present
        if "```" in text:
            if "```json" in text:
                text = text.split("```json")[1].split("```")[0]
            else:
                text = text.split("```")[1].split("```")[0]

        result = json.loads(text.strip())
        print(f"[Vision] Gemini response: {result['selected_item']} (confidence: {result['confidence']})")
        return result

    except Exception as e:
        print(f"[Vision] Gemini API error: {e}, falling back to mock")
        return _mock_response(aisle_info)


def _mock_response(aisle_info):
    """
    Mock vision responses — scientifically accurate for demo testing
    Maps Aisle to most-critical item with reasoning
    """
    responses = {
        "A": {
            "selected_item": "CO2 Fire Extinguisher",
            "position": "leftmost",
            "reasoning": "CO2 displaces oxygen and doesn't conduct electricity—essential for energized electrical fires. Non-conductive discharge prevents equipment damage.",
            "confidence": 0.95,
            "size": "medium",
            "priority": "critical"
        },
        "B": {
            "selected_item": "Hazmat Containment Suit",
            "position": "leftmost",
            "reasoning": "Level A hazmat suit provides vapor-tight protection against chemical vapor exposure. Must don protective equipment before approaching any chemical fire scene.",
            "confidence": 0.93,
            "size": "large",
            "priority": "critical"
        },
        "C": {
            "selected_item": "Advanced Trauma Bag",
            "position": "center-left",
            "reasoning": "Contains hemostatic gauze, tourniquets, and advanced airway management tools. Critical for treating burn injuries and severe trauma from fire incidents.",
            "confidence": 0.94,
            "size": "medium",
            "priority": "high"
        },
        "D": {
            "selected_item": "SCBA Air Tank (30-min rated)",
            "position": "leftmost",
            "reasoning": "Self-Contained Breathing Apparatus with 30-minute clean air supply for IDLH (Immediately Dangerous to Life/Health) atmospheres. Required before firefighter entry.",
            "confidence": 0.97,
            "size": "medium",
            "priority": "critical"
        },
    }

    aisle = aisle_info.get("name", "A")
    response = responses.get(aisle, responses["A"])

    print(f"[Vision] Mock response for Aisle {aisle}: {response['selected_item']} ({response['confidence']:.0%} confidence)")
    return response


# Position mapping: vision response → physical strafe offset
POSITION_OFFSETS = {
    "leftmost":      -0.4,
    "center-left":   -0.2,
    "center":         0.0,
    "center-right":   0.2,
    "rightmost":      0.4,
}


def get_strafe_offset(position_label):
    """
    Convert vision position label to physical strafe distance in meters.
    Negative = left, positive = right
    """
    return POSITION_OFFSETS.get(position_label, 0.0)
