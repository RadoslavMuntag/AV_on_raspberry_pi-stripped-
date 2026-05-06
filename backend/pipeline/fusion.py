from __future__ import annotations

import time
from collections import deque

from .config import PipelineConfig
from ..contracts import PerceptionFrame, WorldState, SensorType

class FusionModule:
    def __init__(self, cfg: PipelineConfig | None = None) -> None:
        self.cfg: PipelineConfig = cfg or PipelineConfig()
        # Initialize rolling window buffers for speed smoothing
        self.left_speed_history: deque[float] = deque(maxlen=self.cfg.speed_window_size)
        self.right_speed_history: deque[float] = deque(maxlen=self.cfg.speed_window_size)
        
    def fuse(self, p: PerceptionFrame) -> WorldState:
        ts = time.monotonic()
        dist = p.ultrasonic_cm
        ultrasonic_close = dist is not None and dist <= self.cfg.obstacle_threshold_cm

        obs_w = p.obstacle_width_cm
        obs_x = float(p.obstacle_x_norm or 0.0)
        frame_width_cm = p.obstacle_frame_width_cm

        clearance_left = 1.0
        clearance_right = 1.0
        obstacle_too_wide = False
        obstacle_width_cm: float | None = None


        if ultrasonic_close and obs_w is not None:
            frame_width_cm = max(1e-6, float(frame_width_cm or self.cfg.obstacle_frame_width_cm))
            half_frame_cm = frame_width_cm * 0.5

            center_cm = obs_x * half_frame_cm
            half_obs_cm = float(obs_w) * 0.5
            left_edge_cm = center_cm - half_obs_cm
            right_edge_cm = center_cm + half_obs_cm

            left_clear_cm = max(0.0, left_edge_cm + half_frame_cm)
            right_clear_cm = max(0.0, half_frame_cm - right_edge_cm)

            clearance_left = max(0.0, min(1.0, left_clear_cm / half_frame_cm))
            clearance_right = max(0.0, min(1.0, right_clear_cm / half_frame_cm))

            # Evaluate passability only on the side with more free clearance.
            if clearance_right >= clearance_left:
                effective_width_cm = max(0.0, right_edge_cm)
            else:
                effective_width_cm = max(0.0, -left_edge_cm)

            obstacle_too_wide = effective_width_cm >= float(self.cfg.obstacle_max_passable_width_cm)
            obstacle_width_cm = float(obs_w)

        obstacle = ultrasonic_close
        stale = (ts - p.ts) > self.cfg.max_sensor_age_s

        # Add current speeds to rolling window and compute smoothed averages
        left_speed_raw = float(p.left_speed or 0.0)
        right_speed_raw = float(p.right_speed or 0.0)
        self.left_speed_history.append(left_speed_raw)
        self.right_speed_history.append(right_speed_raw)
        
        left_speed_smoothed = sum(self.left_speed_history) / len(self.left_speed_history) if self.left_speed_history else 0.0
        right_speed_smoothed = sum(self.right_speed_history) / len(self.right_speed_history) if self.right_speed_history else 0.0

        if self.cfg.DEBUG:
            print("DEBUG: Fusing perception into world state - obstacle:", obstacle, "distance:", dist, "stale:", stale)

        return WorldState(
            ts=ts,
            obstacle_ahead=bool(obstacle),
            obstacle_distance_cm=dist,
            lane_detected=(p.line_offset is not None and p.line_angle is not None),
            line_offset=float(p.line_offset or 0.0),
            line_angle=float(p.line_angle or 0.0),
            line_curvature=float(p.line_curvature or 0.0),

            left_distance=float(p.left_distance or 0.0),
            right_distance=float(p.right_distance or 0.0),

            left_speed=left_speed_smoothed,
            right_speed=right_speed_smoothed,

            obstacle_width_cm=obstacle_width_cm,
            obstacle_clearance_left=clearance_left,
            obstacle_clearance_right=clearance_right,
            obstacle_too_wide=obstacle_too_wide,

            sensor_health={
                SensorType.ULTRASONIC: p.ultrasonic_cm is not None,
                SensorType.INFRARED: False,
                SensorType.CAMERA: p.camera_ok,
                SensorType.LEFT_ENCODER: p.left_speed is not None,
                SensorType.RIGHT_ENCODER: p.right_speed is not None,
            },
            stale=stale,
        )