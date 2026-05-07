from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from dataclasses import fields
from pathlib import Path
from typing import cast

@dataclass(slots=True)
class PipelineConfig:
    # The default values of attributes in this class are not necessarily the defaults used by the system, 
    # but rather serve as type annotations and placeholders. 
    # The actual default values are defined in the JSON config file (e.g., backend/config/pipeline.json) and loaded at runtime. 

    obstacle_threshold_cm: float = 35.0
    max_sensor_age_s: float = 0.25

    # ------------------------------------------------------------------
    # obstacle avoidance parameters
    # ------------------------------------------------------------------

    obstacle_turn_distance_factor: float = 0.5  # multiplier for the number of 90-degree turns to perform when avoiding an obstacle (should be less then 1.0, since robot will likely overshoot a perfect 90-degree turn)
    obstacle_forward_distance_cm: float = 10.0
    obstacle_forward_distance_2_cm: float = 20.0
    obstacle_reacquire_min_forward_cm: float = 0.0
    obstacle_turn_speed: float = 0.13
    obstacle_turn_slow_speed: float = 0.1
    obstacle_forward_speed: float = 0.28
    obstacle_forward_slow_speed: float = 0.12
    obstacle_forward_redetect_hold_s: float = 0.25
    obstacle_reacquire_speed: float = 0.28
    obstacle_reacquire_slow_speed: float = 0.27
    obstacle_slowdown_ratio: float = 0.8

    # ------------------------------------------------------------------
    # obstacle detection parameters
    # ------------------------------------------------------------------

    obstacle_far_roi_ratio: float = 0.45
    obstacle_min_area_px: float = 200.0
    obstacle_px_per_cm: float = 8.0
    camera_diag_fov_deg: float = 75.0
    obstacle_frame_width_cm: float = 30.0
    obstacle_max_passable_width_cm: float = 15.0
    obstacle_adaptive_speed: bool = True

    # ------------------------------------------------------------------
    # line following parameters
    # ------------------------------------------------------------------

    adaptive_speed: bool = False # whether to automatically reduce speed based on curvatre, this feature is experimental 
    cruise_speed: float = 23.0 # in cm/s

    line_threshold: float = 100.0
    line_kp: float = 1.8
    line_angle_kp: float = 1.2
    line_curvature_speed_gain: float = 50.0
    line_min_speed_factor: float = 0.45
    min_confidence: float = 0.25

    # ------------------------------------------------------------------
    # control parameters
    # ------------------------------------------------------------------

    # Speed smoothing parameters
    speed_window_size: int = 5  # number of past speed samples to average for smoothing

    # PID controller parameters
    speed_kp: float = 0.019
    speed_ki: float = 0.001
    speed_kd: float = 0.00001
    integral_limits: tuple[float | None, float | None] = (None, 100.0)  # limits for the integral term in the PID controller to prevent windup

    max_pwm: int = 4096 # maximum PWM value for motor control, in the range [0, 4096]
    min_pwm: int = 0  # minimum PWM to overcome static friction and ensure movement, in the range [0, max_pwm]


    wheel_track: float = 14.0 # distance between tracks in cm, used for kinematic calculations
    wheel_radius: float = 1.25 # radius of the wheels in cm, used for kinematic calculations

    DEBUG: bool = False

    @classmethod
    def from_json_file(cls, json_path: str | Path) -> PipelineConfig:
        path = Path(json_path)
        with path.open("r", encoding="utf-8") as f:
            payload_obj = cast(object, json.load(f))

        if not isinstance(payload_obj, dict):
            raise ValueError("Config JSON root must be an object")
        payload_map = cast(dict[object, object], payload_obj)

        payload: dict[str, object] = {}
        for raw_key, raw_value in payload_map.items():
            if not isinstance(raw_key, str):
                raise TypeError("Config JSON keys must be strings")
            payload[raw_key] = raw_value

        cfg = cls()
        cfg.update_from_mapping(payload)
        return cfg

    def update_from_mapping(self, values: Mapping[str, object]) -> None:
        valid_keys = {f.name for f in fields(self)}
        unknown_keys = [k for k in values.keys() if k not in valid_keys]
        if unknown_keys:
            joined = ", ".join(sorted(unknown_keys))
            raise KeyError(f"Unknown config keys: {joined}")

        for key, raw_value in values.items():
            current = cast(object, getattr(self, key))
            if isinstance(current, bool):
                if not isinstance(raw_value, bool):
                    raise TypeError(f"Config key '{key}' expects bool")
                cast_value: object = raw_value
            elif isinstance(current, int):
                if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
                    raise TypeError(f"Config key '{key}' expects int")
                cast_value = int(raw_value)
            elif isinstance(current, float):
                if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
                    raise TypeError(f"Config key '{key}' expects float")
                cast_value = float(raw_value)
            else:
                cast_value = raw_value

            setattr(self, key, cast_value)



