from __future__ import annotations
from dataclasses import dataclass
from dataclasses import fields
from pathlib import Path
from threading import RLock

from backend.pipeline.controller import DifferentialDriveController
from backend.pipeline.planner import BehaviorPlanner
from backend.pipeline.fusion import FusionModule
from backend.pipeline.config import PipelineConfig
from backend.pipeline.perception import PerceptionModule

from backend.contracts import ManualCommand, PerceptionFrame, PlannerDecision, ControlTargets, WorldState, BehaviorState

from backend.services.hardware import VehicleHardware

@dataclass(slots=True)
class PipelineSnapshot:
    perception: PerceptionFrame
    world: WorldState
    decision: PlannerDecision
    control: ControlTargets


class ModularPipeline:
    def __init__(
        self,
        config: PipelineConfig | None = None,
        perception: PerceptionModule | None = None,
        fusion: FusionModule | None = None,
        planner: BehaviorPlanner | None = None,
        controller: DifferentialDriveController | None = None,
    ) -> None:
        self.config : PipelineConfig = config or PipelineConfig()
        self._config_lock: RLock = RLock()
        self.perception: PerceptionModule = perception or PerceptionModule(cfg=self.config)
        self.fusion: FusionModule = fusion or FusionModule(cfg=self.config)
        self.planner: BehaviorPlanner = planner or BehaviorPlanner(cfg=self.config)
        self.controller: DifferentialDriveController = controller or DifferentialDriveController(cfg=self.config)

        if config is None:
            try:
                self.load_config_from_json("backend/config/pipeline.json")
            except Exception as e:
                print(f"Error loading pipeline config: {e}, falling back to defaults")
                

    def load_config_from_json(self, json_path: str | Path) -> PipelineConfig:
        loaded = PipelineConfig.from_json_file(json_path)
        with self._config_lock:
            for f in fields(PipelineConfig):
                setattr(self.config, f.name, getattr(loaded, f.name))
                
            self.controller.tune_pid(
                kp=self.config.speed_kp,
                ki=self.config.speed_ki,
                kd=self.config.speed_kd
            )
            return self.config

    def tick(
        self,
        hardware: VehicleHardware,
        requested_mode: BehaviorState,
        heartbeat_ok: bool,
        manual_cmd: ManualCommand,
        dt: float
    ) -> PipelineSnapshot:
        with self._config_lock:
            p : PerceptionFrame = self.perception.read(hardware, dt)
            w : WorldState = self.fusion.fuse(p)
            d : PlannerDecision = self.planner.step(w, requested_mode=requested_mode, heartbeat_ok=heartbeat_ok)
            u : ControlTargets = self.controller.step(d, w, manual_cmd, dt)

            if self.config.DEBUG:
                print("DEBUG: Control outputs - left_pwm:", u.left_pwm, "right_pwm:", u.right_pwm, "led_mode:", u.led_mode, "led_rgb:", u.led_rgb)

            hardware.set_motor(u.left_pwm, u.right_pwm)
            hardware.set_led(u.led_mode, u.led_rgb[0], u.led_rgb[1], u.led_rgb[2], 0)

            return PipelineSnapshot(perception=p, world=w, decision=d, control=u)