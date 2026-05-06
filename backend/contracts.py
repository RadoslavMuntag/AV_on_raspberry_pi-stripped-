from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

class InfraredState(int, Enum):
    # The infrared.py module returns a single integer representing the state of the three infrared sensors, 
    # where each bit corresponds to a sensor (1 for active, 0 for inactive). 
    # The possible states are:
    NO_DETECTION = 0
    RIGHT = 1
    MIDDLE = 2
    RIGHT_MIDDLE = 3
    LEFT = 4
    LEFT_RIGHT = 5
    LEFT_MIDDLE = 6
    ALL = 7
    
class BehaviorState(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    LINE_FOLLOW = "line_follow"
    OBSTACLE_AVOID = "obstacle_avoid"
    SAFE_STOP = "safe_stop"

class LedMode(str, Enum):
    OFF = "off"
    INDEX = "index"
    BLINK = "blink"
    BREATHING = "breathing"
    RAINBOW = "rainbow"

class SensorType(str, Enum):
    ULTRASONIC = "ultrasonic"
    INFRARED = "infrared"
    CAMERA = "camera"
    LEFT_ENCODER = "left_encoder"
    RIGHT_ENCODER = "right_encoder"


class ObstacleAvoidPhase(str, Enum):
    TURN_OUT = "turn_out"
    DRIVE_FORWARD_1 = "drive_forward_1"
    TURN_BACK_1 = "turn_back_1"
    DRIVE_FORWARD_2 = "drive_forward_2"
    TURN_BACK_2 = "turn_back_2"
    REACQUIRE_FORWARD = "reacquire_forward"

@dataclass(slots=True)
class ManualCommand:
    throttle: float = 0.0  # [-1.0, 1.0]
    steer: float = 0.0     # [-1.0, 1.0]
    active: bool = False   # TODO: Dont know if bool is needed, consider removing

# ts <=> timestamp
@dataclass(slots=True)
class PerceptionFrame:
    ts: float
    ultrasonic_cm: float | None

    left_distance: float | None
    right_distance: float | None

    left_speed: float | None = None
    right_speed: float | None = None

    line_angle: float | None = None  # in radians
    line_curvature: float | None = None  # in 1/m
    line_offset: float | None = None  # in range [-1, 1], normalized from pixels
    line_confidence: float = 0.0  # in range [0, 1]
    obstacle_width_cm: float | None = None
    obstacle_x_norm: float | None = None
    obstacle_frame_width_cm: float | None = None

    camera_ok: bool = True
    faults: list[str] = field(default_factory=list)


@dataclass(slots=True)
class WorldState:
    ts: float
    obstacle_ahead: bool
    obstacle_distance_cm: float | None
    lane_detected: bool
    
    line_offset: float
    line_angle: float
    line_curvature: float

    left_distance: float
    right_distance: float

    left_speed: float
    right_speed: float

    obstacle_width_cm: float | None = None
    obstacle_clearance_left: float = 1.0
    obstacle_clearance_right: float = 1.0
    obstacle_too_wide: bool = False

    sensor_health: dict[SensorType, bool] = field(default_factory=dict)
    stale: bool = False


@dataclass(slots=True)
class PlannerDecision:
    ts: float
    state: BehaviorState
    reason: str
    desired_speed: float  # [0, 1]
    desired_turn: float   # [-1, 1]
    safe_stop: bool = False
    avoid_phase: ObstacleAvoidPhase | None = None
    avoid_progress_cm: float | None = None
    avoid_target_cm: float | None = None


@dataclass(slots=True)
class ControlTargets:
    ts: float
    left_pwm: int
    right_pwm: int
    led_mode: LedMode = LedMode.OFF
    led_rgb: tuple[int, int, int] = (0, 0, 0)