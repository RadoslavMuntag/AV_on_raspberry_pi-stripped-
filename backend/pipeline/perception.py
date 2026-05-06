from __future__ import annotations

import time


from .config import PipelineConfig
from ..contracts import PerceptionFrame
from ..services.hardware import VehicleHardware
from .vision import detect_line_geometry

class PerceptionModule:
    def __init__(self, cfg: PipelineConfig | None = None) -> None:
        self.cfg = cfg or PipelineConfig()

    def read(self, hardware: VehicleHardware, dt: float) -> PerceptionFrame:
        ts = time.monotonic()
        faults: list[str] = []

        ultrasonic = hardware.read_ultrasonic()
        if ultrasonic is None:
            faults.append("ultrasonic_unavailable")
        
        left_distance = hardware.read_left_encoder_distance(self.cfg.wheel_radius)
        left_speed = hardware.read_left_encoder(self.cfg.wheel_radius, dt)
        if left_speed is None:
            faults.append("left_encoder_unavailable")

        right_distance = hardware.read_right_encoder_distance(self.cfg.wheel_radius)
        right_speed = hardware.read_right_encoder(self.cfg.wheel_radius, dt)
        if right_speed is None:
            faults.append("right_encoder_unavailable")

        camera_frame = hardware.get_jpeg_frame()
        cam_offset: float | None = None
        cam_angle: float | None = None
        cam_curvature: float | None = None
        cam_confidence: float = 0.0
        cam_obstacle_width_cm: float | None = None
        cam_obstacle_x_norm: float | None = None
        cam_obstacle_frame_width_cm: float | None = None

        angle, curvature, offset, confidence = None, None, None, 0.0
        if camera_frame is None:
            faults.append("camera_unavailable")
        else:
            try:
                (
                    cam_angle,
                    cam_curvature,
                    cam_offset,
                    cam_confidence,
                    cam_obstacle_width_cm,
                    cam_obstacle_x_norm,
                    cam_obstacle_frame_width_cm,
                    debug,
                ) = detect_line_geometry(
                    camera_frame,
                    obstacle_far_roi_ratio=self.cfg.obstacle_far_roi_ratio,
                    obstacle_min_area_px=self.cfg.obstacle_min_area_px,
                    obstacle_px_per_cm=self.cfg.obstacle_px_per_cm,
                    obstacle_distance_cm=ultrasonic,
                    camera_diag_fov_deg=self.cfg.camera_diag_fov_deg,
                    threshold=self.cfg.line_threshold,
                )
                hardware.set_debug_frame(debug)
            except Exception as e:
                faults.append(f"line_geometryppp_error: {str(e)}")

            if cam_confidence >= self.cfg.min_confidence:    
                angle = cam_angle
                curvature = cam_curvature
                offset = cam_offset
            confidence = cam_confidence
        
        if self.cfg.DEBUG:
            print("DEBUG: Perception read - ultrasonic:", ultrasonic, "left_speed:", left_speed, "right_speed:", right_speed, "angle:", angle, "curvature:", curvature, "offset:", offset, "faults:", faults)

        return PerceptionFrame(
            ts=ts,
            ultrasonic_cm=ultrasonic,

            left_distance=left_distance,
            right_distance=right_distance,

            left_speed=left_speed,
            right_speed=right_speed,

            line_angle=angle,
            line_curvature=curvature,
            line_offset=offset,
            line_confidence=confidence,
            obstacle_width_cm=cam_obstacle_width_cm,
            obstacle_x_norm=cam_obstacle_x_norm,
            obstacle_frame_width_cm=cam_obstacle_frame_width_cm,
            camera_ok=hardware.ready,
            faults=faults,
        )
