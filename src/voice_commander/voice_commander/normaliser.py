"""
Deterministic normalisation layer.

The LLM produces a *semantic intent*: JSON in human terms
("turn", 90, "degrees", "left"). This module is the ONLY path from that
intent to a robot command. It is pure Python (no ROS, no AI), so it is
deterministic and unit-testable.

Output conventions (REP-103): distances in metres, angles in radians,
+yaw = counter-clockwise = LEFT, + distance = forward.

Policy: anything malformed, ambiguous, unsupported or out of bounds is
REJECTED (never guessed or silently clamped).
"""
import json
import math
import re
from dataclasses import dataclass
from typing import Literal, Optional

from pydantic import BaseModel, ValidationError, field_validator

MAX_DISTANCE_M = 20.0
MAX_YAW_RAD = 2 * math.pi

DISTANCE_TO_M = {
    "m": 1.0, "meter": 1.0, "meters": 1.0, "metre": 1.0, "metres": 1.0,
    "cm": 0.01, "centimeter": 0.01, "centimeters": 0.01,
    "mm": 0.001, "millimeter": 0.001, "millimeters": 0.001,
    "ft": 0.3048, "foot": 0.3048, "feet": 0.3048,
    "in": 0.0254, "inch": 0.0254, "inches": 0.0254,
}
ANGLE_TO_RAD = {
    "deg": math.pi / 180, "degree": math.pi / 180, "degrees": math.pi / 180,
    "rad": 1.0, "radian": 1.0, "radians": 1.0,
}
FORWARD = {"forward", "forwards", "ahead", "straight"}
BACKWARD = {"backward", "backwards", "back", "reverse"}
LEFT = {"left", "counterclockwise", "counter-clockwise", "anticlockwise", "ccw"}
RIGHT = {"right", "clockwise", "cw"}


class NormalisationError(Exception):
    """Raised when an intent cannot be safely turned into a robot command."""


@dataclass(frozen=True)
class RobotCommand:
    kind: str                 # "move" | "turn" | "stop"
    distance_m: float = 0.0   # signed, + = forward
    yaw_rad: float = 0.0      # signed, + = left (CCW)

    def __str__(self):
        return f"RobotCommand(kind={self.kind}, distance_m={self.distance_m:.3f}, yaw_rad={self.yaw_rad:.4f})"


class Intent(BaseModel):
    """Schema the LLM must follow."""
    action: Literal["move", "turn", "stop", "unsupported"]
    distance: Optional[float] = None
    distance_unit: Optional[str] = None
    direction: Optional[str] = None
    angle: Optional[float] = None
    angle_unit: Optional[str] = None

    @field_validator("action", "distance_unit", "direction", "angle_unit", mode="before")
    @classmethod
    def _lower(cls, v):
        return v.strip().lower() if isinstance(v, str) else v


def extract_json(text: str) -> dict:
    """Pull the first JSON object out of raw LLM text (handles ``` fences and chatter)."""
    text = re.sub(r"```(?:json)?", "", text)
    start, end = text.find("{"), text.rfind("}")
    if start == -1 or end <= start:
        raise NormalisationError("no JSON object found in LLM output")
    try:
        data = json.loads(text[start:end + 1])
    except json.JSONDecodeError as e:
        raise NormalisationError(f"invalid JSON: {e.msg}")
    if not isinstance(data, dict):
        raise NormalisationError("LLM output is not a JSON object")
    return data


def _finite_positive(value, name):
    if value is None:
        raise NormalisationError(f"missing {name}")
    if not math.isfinite(value):
        raise NormalisationError(f"{name} is not finite")
    if value <= 0:
        raise NormalisationError(f"{name} must be > 0 (direction is given separately)")
    return value


def normalise(llm_output) -> RobotCommand:
    """LLM text or dict -> validated RobotCommand, or raise NormalisationError."""
    data = extract_json(llm_output) if isinstance(llm_output, str) else llm_output
    try:
        intent = Intent.model_validate(data)
    except ValidationError as e:
        raise NormalisationError(f"malformed intent: {e.errors()[0]['msg']}")

    if intent.action == "stop":
        return RobotCommand("stop")

    if intent.action == "unsupported":
        raise NormalisationError("command not supported by this robot")

    if intent.action == "move":
        magnitude = _finite_positive(intent.distance, "distance")
        if intent.distance_unit not in DISTANCE_TO_M:
            raise NormalisationError(f"unknown/missing distance unit: {intent.distance_unit!r}")
        if intent.direction is None or intent.direction in FORWARD:
            sign = 1.0
        elif intent.direction in BACKWARD:
            sign = -1.0
        else:
            raise NormalisationError(f"invalid direction for move: {intent.direction!r}")
        distance_m = sign * magnitude * DISTANCE_TO_M[intent.distance_unit]
        if abs(distance_m) > MAX_DISTANCE_M:
            raise NormalisationError(f"|distance| {abs(distance_m):.2f} m exceeds limit {MAX_DISTANCE_M} m")
        return RobotCommand("move", distance_m=distance_m)

    # turn
    magnitude = _finite_positive(intent.angle, "angle")
    if intent.angle_unit not in ANGLE_TO_RAD:
        raise NormalisationError(f"unknown/missing angle unit: {intent.angle_unit!r}")
    if intent.direction in LEFT:
        sign = 1.0     # REP-103: left = CCW = +yaw
    elif intent.direction in RIGHT:
        sign = -1.0
    else:
        raise NormalisationError(f"turn needs direction left/right, got {intent.direction!r}")
    yaw_rad = sign * magnitude * ANGLE_TO_RAD[intent.angle_unit]
    if abs(yaw_rad) > MAX_YAW_RAD:
        raise NormalisationError(f"|yaw| {abs(yaw_rad):.2f} rad exceeds limit {MAX_YAW_RAD:.2f} rad")
    return RobotCommand("turn", yaw_rad=yaw_rad)


if __name__ == "__main__":
    # Self-test / demo: shows every normalisation rule on example LLM outputs.
    cases = [
        ("Scenario 2: turn 90 deg left",
         '{"action": "turn", "angle": 90, "angle_unit": "degrees", "direction": "left"}'),
        ("turn right 45 deg",
         '{"action": "turn", "angle": 45, "angle_unit": "deg", "direction": "right"}'),
        ("move forward 10 m",
         '{"action": "move", "distance": 10, "distance_unit": "meters", "direction": "forward"}'),
        ("move back 3 feet",
         '{"action": "move", "distance": 3, "distance_unit": "feet", "direction": "backward"}'),
        ("stop", '{"action": "stop"}'),
        ("LLM chatter + code fence",
         'Sure! ```json\n{"action": "Move", "distance": "2", "distance_unit": "M"}\n```'),
        ("truncated JSON", '{"action": "move", "distance": 10,'),
        ("out of bounds (500 m)",
         '{"action": "move", "distance": 500, "distance_unit": "m", "direction": "forward"}'),
        ("missing unit",
         '{"action": "move", "distance": 5, "direction": "forward"}'),
        ("turn without direction",
         '{"action": "turn", "angle": 90, "angle_unit": "degrees"}'),
        ("unsupported command", '{"action": "unsupported"}'),
        ("unknown action", '{"action": "dance"}'),
    ]
    for label, raw in cases:
        try:
            print(f"OK      {label:30s} -> {normalise(raw)}")
        except NormalisationError as e:
            print(f"REJECT  {label:30s} -> {e}")