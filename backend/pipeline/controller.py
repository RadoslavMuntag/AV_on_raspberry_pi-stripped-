from __future__ import annotations

import time

from backend.pipeline.config import PipelineConfig
from backend.misc.PID import SpeedPIDController
from backend.contracts import BehaviorState, ControlTargets, LedMode, ManualCommand, PlannerDecision, ObstacleAvoidPhase, WorldState

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class DifferentialDriveController:
    def __init__(self, cfg: PipelineConfig | None = None) -> None:
        self.cfg: PipelineConfig = cfg or PipelineConfig()
        self.last_state : BehaviorState = BehaviorState.IDLE
        self.last_phase: ObstacleAvoidPhase | None = None
        
        self.left_pid: SpeedPIDController = SpeedPIDController(
            kp=self.cfg.speed_kp,
            ki=self.cfg.speed_ki,
            kd=self.cfg.speed_kd
        )
        self.right_pid: SpeedPIDController = SpeedPIDController(
            kp=self.cfg.speed_kp,
            ki=self.cfg.speed_ki,
            kd=self.cfg.speed_kd
        )

    def tune_pid(self, kp: float | None = None, ki: float | None = None, kd: float | None = None) -> None:
        self.left_pid.tune(kp, ki, kd)
        self.right_pid.tune(kp, ki, kd)

    def _inverse_differential_kinematics(self, linear: float, angular: float) -> tuple[float, float]:
        vl = linear - angular * self.cfg.wheel_track / 2
        vr = linear + angular * self.cfg.wheel_track / 2

        return vl, vr

    def _mix(self, speed: float, turn: float) -> tuple[int, int]:
        speed = _clamp(speed, -1.0, 1.0)
        turn = _clamp(turn, -1.0, 1.0)
        left = int((speed - turn) * self.cfg.max_pwm)
        right = int((speed + turn) * self.cfg.max_pwm)
        return left, right

    def _map_gain_to_pwm(self, gain: float) -> int:
        gain = _clamp(gain, -1.0, 1.0)
        # Implement a deadzone around zero to prevent oscillation and ensure the robot can stop effectively
        if abs(gain) < 0.005:
            return 0  # deadzone to prevent oscillation around zero speed
        dir = gain / abs(gain)
        return int(gain * (self.cfg.max_pwm - self.cfg.min_pwm) + self.cfg.min_pwm * dir)

    def step(
        self,
        decision: PlannerDecision,
        world: WorldState,
        manual: ManualCommand,
        dt: float,
    ) -> ControlTargets:
        now = time.monotonic()
        
        if self.last_state != decision.state or decision.avoid_phase != self.last_phase:
            self.left_pid.reset()
            self.right_pid.reset()
            self.last_state = decision.state
            self.last_phase = decision.avoid_phase

        if decision.safe_stop or decision.state == BehaviorState.SAFE_STOP:
            # Stop immediately, blink red light to indicate E-stop
            return ControlTargets(now, 0, 0, LedMode.BLINK, (255, 0, 0))

        if decision.state == BehaviorState.IDLE:
            # No control output, no lights
            self.left_pid.reset()
            self.right_pid.reset()
            return ControlTargets(now, 0, 0, LedMode.OFF, (0, 0, 0))

        if decision.state == BehaviorState.MANUAL:
            # If not active stop, otherwise use manual command, solid orange light
            self.left_pid.reset()
            self.right_pid.reset()
            if not manual.active:
                return ControlTargets(now, 0, 0, LedMode.INDEX, (255, 120, 0))
            left, right = self._mix(manual.throttle, manual.steer)
            return ControlTargets(now, left, right, LedMode.INDEX, (0, 120, 255))

        if decision.state == BehaviorState.LINE_FOLLOW:
            #return ControlTargets(now, 0, 0, LedMode.INDEX, (255, 255, 0))
            #if world.lane_detected and world.lateral_confidence >= self.cfg.min_confidence:
            if decision.desired_speed == 0.0:
                return ControlTargets(now, 0, 0, LedMode.INDEX, (255, 255, 0))

            vl, vr = self._inverse_differential_kinematics(decision.desired_speed, decision.desired_turn)

            self.left_pid.set_setpoint(vl)
            self.right_pid.set_setpoint(vr)

            gain_left, gain_right = self.left_pid.update(world.left_speed, dt), self.right_pid.update(world.right_speed, dt)
            left, right = self._map_gain_to_pwm(gain_left), self._map_gain_to_pwm(gain_right)

            return ControlTargets(now, left, right, LedMode.INDEX, (0, 255, 0))
            #return ControlTargets(now, 0, 0, "blink", (255, 255, 0))

        if decision.state == BehaviorState.OBSTACLE_AVOID:
            left, right = self._mix(decision.desired_speed, decision.desired_turn)
            return ControlTargets(now, left, right, LedMode.INDEX, (255, 0, 255))

        return ControlTargets(now, 0, 0, LedMode.OFF, (0, 0, 0))