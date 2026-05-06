from __future__ import annotations

import math
import time

from backend.pipeline.config import PipelineConfig
from backend.contracts import (
    BehaviorState,
    ObstacleAvoidPhase,
    PlannerDecision,
    WorldState,
    SensorType,
)


class BehaviorPlanner:
    """
    Behaviour planner with an obstacle-avoidance sequence.
    
    """

    # Wall-clock budget per phase in seconds.
    # If a phase takes longer than this, it is force-advanced 
    _PHASE_TIMEOUT_S: dict[ObstacleAvoidPhase, float] = {
        ObstacleAvoidPhase.TURN_OUT:         4.0,
        ObstacleAvoidPhase.DRIVE_FORWARD_1:  5.0,
        ObstacleAvoidPhase.TURN_BACK_1:      4.0,
        ObstacleAvoidPhase.DRIVE_FORWARD_2:  5.0,
        ObstacleAvoidPhase.TURN_BACK_2:      4.0,
        ObstacleAvoidPhase.REACQUIRE_FORWARD: 6.0,
    }

    def __init__(self, cfg: PipelineConfig | None = None) -> None:
        self.cfg: PipelineConfig = cfg or PipelineConfig()
        self.current_state: BehaviorState = BehaviorState.IDLE

        self._avoid_phase: ObstacleAvoidPhase | None = None
        self._avoid_start_left: float | None = None
        self._avoid_start_right: float | None = None
        self._avoid_phase_start_time: float | None = None
        self._forward_redetect_hold_start_time: float | None = None

        # +1.0 = turn right, -1.0 = turn left
        self._avoid_turn_dir: float = 1.0

    # ------------------------------------------------------------------
    # Avoidance sequence helpers
    # ------------------------------------------------------------------

    def _reset_avoid_sequence(self) -> None:
        self._avoid_phase = None
        self._avoid_start_left = None
        self._avoid_start_right = None
        self._avoid_phase_start_time = None
        self._forward_redetect_hold_start_time = None

    def _start_avoid_phase(self, phase: ObstacleAvoidPhase, world: WorldState) -> None:
        self._avoid_phase = phase
        self._avoid_start_left = world.left_distance
        self._avoid_start_right = world.right_distance
        self._avoid_phase_start_time = time.monotonic()
        self._forward_redetect_hold_start_time = None

    def _choose_turn_direction(self, world: WorldState) -> float | None:
        """
        Returns +1.0 (right), -1.0 (left), or None when no passage is available.
        """
        if world.obstacle_too_wide:
            return None

        if world.obstacle_clearance_right >= world.obstacle_clearance_left:
            return -1.0
        return 1.0

    def _advance_avoid_sequence(self, world: WorldState) -> None:
        _next: dict[ObstacleAvoidPhase, ObstacleAvoidPhase | None] = {
            ObstacleAvoidPhase.TURN_OUT:          ObstacleAvoidPhase.DRIVE_FORWARD_1,
            ObstacleAvoidPhase.DRIVE_FORWARD_1:   ObstacleAvoidPhase.TURN_BACK_1,
            ObstacleAvoidPhase.TURN_BACK_1:       ObstacleAvoidPhase.DRIVE_FORWARD_2,
            ObstacleAvoidPhase.DRIVE_FORWARD_2:   ObstacleAvoidPhase.TURN_BACK_2,
            ObstacleAvoidPhase.TURN_BACK_2:       ObstacleAvoidPhase.REACQUIRE_FORWARD,
            ObstacleAvoidPhase.REACQUIRE_FORWARD: None,
        }
        if self._avoid_phase is None:
            return
        nxt = _next.get(self._avoid_phase)
        if nxt is None:
            self._reset_avoid_sequence()
        else:
            self._start_avoid_phase(nxt, world)

    def _encoder_feedback_ok(self, world: WorldState) -> bool:
        return (
            world.sensor_health.get(SensorType.LEFT_ENCODER, False)
            and world.sensor_health.get(SensorType.RIGHT_ENCODER, False)
        )

    def _phase_progress_cm(self, world: WorldState) -> float | None:
        if self._avoid_start_left is None or self._avoid_start_right is None:
            return None
        left_delta  = abs(world.left_distance  - self._avoid_start_left)
        right_delta = abs(world.right_distance - self._avoid_start_right)
        return (left_delta + right_delta) / 2.0

    def _phase_elapsed_s(self) -> float:
        if self._avoid_phase_start_time is None:
            return 0.0
        return time.monotonic() - self._avoid_phase_start_time

    def _phase_timed_out(self) -> bool:
        if self._avoid_phase is None:
            return False
        limit = self._PHASE_TIMEOUT_S.get(self._avoid_phase, 5.0)
        return self._phase_elapsed_s() >= limit

    def _turn_target_cm(self) -> float:
        return self.cfg.wheel_track * math.pi * 0.25 * self.cfg.obstacle_turn_distance_factor

    def _turn_direction_for_phase(self, phase: ObstacleAvoidPhase) -> float:
        """
        TURN_OUT uses the chosen direction; TURN_BACK phases mirror it
        so the vehicle always returns to the lane regardless of which
        way it initially turned.
        """
        if phase == ObstacleAvoidPhase.TURN_OUT:
            return self._avoid_turn_dir
        # TURN_BACK_1 and TURN_BACK_2 reverse back toward the lane
        return -self._avoid_turn_dir

    # ------------------------------------------------------------------
    # Decision builders
    # ------------------------------------------------------------------

    def _build_decision(
        self,
        now: float,
        state: BehaviorState,
        reason: str,
        desired_speed: float,
        desired_turn: float,
        *,
        avoid_phase: ObstacleAvoidPhase | None = None,
        avoid_progress_cm: float | None = None,
        avoid_target_cm: float | None = None,
        safe_stop: bool = False,
    ) -> PlannerDecision:
        return PlannerDecision(
            ts=now,
            state=state,
            reason=reason,
            desired_speed=desired_speed,
            desired_turn=desired_turn,
            safe_stop=safe_stop,
            avoid_phase=avoid_phase,
            avoid_progress_cm=avoid_progress_cm,
            avoid_target_cm=avoid_target_cm,
        )

    def _decision_safe_stop(self, now: float, reason: str) -> PlannerDecision:
        self._reset_avoid_sequence()
        self.current_state = BehaviorState.SAFE_STOP
        return self._build_decision(now, self.current_state, reason, 0.0, 0.0, safe_stop=True)

    def _decision_manual(self, now: float) -> PlannerDecision:
        self._reset_avoid_sequence()
        self.current_state = BehaviorState.MANUAL
        return self._build_decision(now, self.current_state, "manual_mode", 0.0, 0.0)

    def _decision_idle(self, now: float) -> PlannerDecision:
        self._reset_avoid_sequence()
        self.current_state = BehaviorState.IDLE
        return self._build_decision(now, self.current_state, "idle_mode", 0.0, 0.0)

    # ------------------------------------------------------------------
    # Per-phase handlers
    # ------------------------------------------------------------------

    def _handle_turn_phase(
        self,
        now: float,
        world: WorldState,
        phase: ObstacleAvoidPhase,
        reason: str,
    ) -> PlannerDecision:
        target_cm   = self._turn_target_cm()
        progress_cm = self._phase_progress_cm(world)

        if progress_cm is None:
            return self._decision_safe_stop(now, "avoidance_state_lost")

        timed_out = self._phase_timed_out()

        if progress_cm >= target_cm or timed_out:
            if timed_out and self.cfg.DEBUG:
                print(f"DEBUG: Phase {reason} timed out at progress={progress_cm:.1f} cm")
            self._advance_avoid_sequence(world)
            return self._build_decision(
                now, BehaviorState.OBSTACLE_AVOID, f"{reason}_complete",
                0.0, 0.0,
                avoid_phase=self._avoid_phase,
                avoid_progress_cm=progress_cm,
                avoid_target_cm=target_cm,
            )

        turn_gain = self.cfg.obstacle_turn_speed * self._turn_direction_for_phase(phase)
        if progress_cm >= target_cm * self.cfg.obstacle_slowdown_ratio:
            turn_gain = self.cfg.obstacle_turn_slow_speed * self._turn_direction_for_phase(phase)

        self.current_state = BehaviorState.OBSTACLE_AVOID
        return self._build_decision(
            now, self.current_state, reason, 0.0, turn_gain,
            avoid_phase=self._avoid_phase,
            avoid_progress_cm=progress_cm,
            avoid_target_cm=target_cm,
        )

    def _handle_drive_forward_phase(
        self,
        now: float,
        world: WorldState,
        target_cm: float,
        reason: str,
        speed: float,
    ) -> PlannerDecision:
        progress_cm = self._phase_progress_cm(world)

        if progress_cm is None:
            return self._decision_safe_stop(now, "avoidance_state_lost")

        # --- Re-detection while driving forward --------------------------------
        # Hold briefly to let ultrasonic settle after turn-out transients.
        # If still blocked after the hold, safe-stop instead of re-turning.
        if world.obstacle_ahead:
            hold_s = max(0.0, float(self.cfg.obstacle_forward_redetect_hold_s))
            if self._forward_redetect_hold_start_time is None:
                self._forward_redetect_hold_start_time = now
                if self.cfg.DEBUG:
                    print(f"DEBUG: Obstacle seen during {reason}, holding for {hold_s:.2f}s")
                self.current_state = BehaviorState.OBSTACLE_AVOID
                return self._build_decision(
                    now, self.current_state, "obstacle_redetect_hold",
                    0.0, 0.0,
                    avoid_phase=self._avoid_phase,
                    avoid_progress_cm=progress_cm,
                    avoid_target_cm=target_cm,
                )

            if (now - self._forward_redetect_hold_start_time) < hold_s:
                self.current_state = BehaviorState.OBSTACLE_AVOID
                return self._build_decision(
                    now, self.current_state, "obstacle_redetect_hold",
                    0.0, 0.0,
                    avoid_phase=self._avoid_phase,
                    avoid_progress_cm=progress_cm,
                    avoid_target_cm=target_cm,
                )

            if self.cfg.DEBUG:
                print(f"DEBUG: Obstacle persisted after hold during {reason}, safe-stopping")
            return self._decision_safe_stop(now, "obstacle_redetected_during_forward")
        else:
            self._forward_redetect_hold_start_time = None

        timed_out = self._phase_timed_out()

        if progress_cm >= target_cm or timed_out:
            if timed_out and self.cfg.DEBUG:
                print(f"DEBUG: Phase {reason} timed out at progress={progress_cm:.1f} cm")
            self._advance_avoid_sequence(world)
            return self._build_decision(
                now, BehaviorState.OBSTACLE_AVOID, f"{reason}_complete",
                0.0, 0.0,
                avoid_phase=self._avoid_phase,
                avoid_progress_cm=progress_cm,
                avoid_target_cm=target_cm,
            )

        if progress_cm >= target_cm * self.cfg.obstacle_slowdown_ratio:
            speed = self.cfg.obstacle_forward_slow_speed

        self.current_state = BehaviorState.OBSTACLE_AVOID
        return self._build_decision(
            now, self.current_state, reason, speed, 0.0,
            avoid_phase=self._avoid_phase,
            avoid_progress_cm=progress_cm,
            avoid_target_cm=target_cm,
        )

    def _handle_reacquire_phase(self, now: float, world: WorldState) -> PlannerDecision:
        progress_cm = self._phase_progress_cm(world)

        if progress_cm is None:
            return self._decision_safe_stop(now, "avoidance_state_lost")

        # Success: minimum distance covered AND lane visible
        min_cm = float(self.cfg.obstacle_reacquire_min_forward_cm)
        if progress_cm >= min_cm and world.lane_detected:
            self._reset_avoid_sequence()
            return self._line_follow_nominal(now, world)

        # Safety: new obstacle ahead while reacquiring → stop rather than collide
        if world.obstacle_ahead:
            return self._decision_safe_stop(now, "obstacle_during_reacquire")

        # Timeout: lane never found — safe stop to avoid driving off into the void
        if self._phase_timed_out():
            return self._decision_safe_stop(now, "reacquire_timeout_lane_not_found")

        speed = self.cfg.obstacle_reacquire_speed
        if progress_cm >= min_cm * self.cfg.obstacle_slowdown_ratio:
            speed = self.cfg.obstacle_reacquire_slow_speed

        self.current_state = BehaviorState.OBSTACLE_AVOID
        return self._build_decision(
            now, self.current_state, "reacquire_forward", speed, 0.0,
            avoid_phase=self._avoid_phase,
            avoid_progress_cm=progress_cm,
            avoid_target_cm=min_cm,
        )

    def _handle_active_avoid_sequence(
        self,
        now: float,
        world: WorldState,
    ) -> PlannerDecision | None:
        if self._avoid_phase is None:
            return None

        if not self._encoder_feedback_ok(world):
            return self._decision_safe_stop(now, "encoder_feedback_unavailable")

        if self._phase_progress_cm(world) is None:
            return self._decision_safe_stop(now, "avoidance_state_lost")

        p = self._avoid_phase

        # In the final manoeuvre phases, a reliable lane reacquire is enough to
        # finish avoidance early and return to nominal line following.
        if p == ObstacleAvoidPhase.TURN_BACK_2 and world.lane_detected:
            if self.cfg.DEBUG:
                print(f"DEBUG: Lane detected during {p}; ending avoidance sequence early")
            self._reset_avoid_sequence()
            return self._line_follow_nominal(now, world)

        if p == ObstacleAvoidPhase.TURN_OUT:
            return self._handle_turn_phase(now, world, p, "turn_out")

        if p == ObstacleAvoidPhase.DRIVE_FORWARD_1:
            return self._handle_drive_forward_phase(
                now, world, float(self.cfg.obstacle_forward_distance_cm),
                "drive_forward_1", self.cfg.obstacle_forward_speed,
            )

        if p == ObstacleAvoidPhase.TURN_BACK_1:
            return self._handle_turn_phase(now, world, p, "turn_back_1")

        if p == ObstacleAvoidPhase.DRIVE_FORWARD_2:
            return self._handle_drive_forward_phase(
                now, world, float(self.cfg.obstacle_forward_distance_2_cm),
                "drive_forward_2", self.cfg.obstacle_forward_speed,
            )

        if p == ObstacleAvoidPhase.TURN_BACK_2:
            return self._handle_turn_phase(now, world, p, "turn_back_2")

        if p == ObstacleAvoidPhase.REACQUIRE_FORWARD:
            return self._handle_reacquire_phase(now, world)

        return None

    # ------------------------------------------------------------------
    # Line-follow helpers
    # ------------------------------------------------------------------

    def _line_follow_nominal(self, now: float, world: WorldState) -> PlannerDecision:
        self.current_state = BehaviorState.LINE_FOLLOW
        if not world.lane_detected:
            return self._build_decision(now, self.current_state, "line_lost", 0.0, 0.0)

        speed = self.cfg.cruise_speed
        if self.cfg.adaptive_speed:
            speed = self.cfg.cruise_speed * max(
                self.cfg.line_min_speed_factor,
                1.0 - self.cfg.line_curvature_speed_gain * abs(world.line_curvature),
            )
        turn = (
            self.cfg.line_kp      * (-world.line_offset)
            + self.cfg.line_angle_kp * world.line_angle
        )
        return self._build_decision(now, self.current_state, "line_follow_nominal", speed, turn)

    def _handle_line_follow_mode(self, now: float, world: WorldState) -> PlannerDecision:
        if world.obstacle_ahead:
            # Choose turn direction NOW, before entering the sequence
            direction = self._choose_turn_direction(world)
            if direction is None:
                return self._decision_safe_stop(now, "obstacle_too_wide_no_passage")
            self._avoid_turn_dir = direction
            self._start_avoid_phase(ObstacleAvoidPhase.TURN_OUT, world)
            self.current_state = BehaviorState.OBSTACLE_AVOID
            return self._build_decision(
                now, self.current_state, "obstacle_detected",
                0.0, self.cfg.obstacle_turn_speed * self._avoid_turn_dir,
                avoid_phase=self._avoid_phase,
                avoid_progress_cm=0.0,
                avoid_target_cm=self._turn_target_cm(),
            )
        return self._line_follow_nominal(now, world)

    def _handle_obstacle_avoid_mode(self, now: float, world: WorldState) -> PlannerDecision:
        self.current_state = BehaviorState.OBSTACLE_AVOID
        if world.obstacle_ahead and self._avoid_phase is None:
            direction = self._choose_turn_direction(world)
            if direction is None:
                return self._decision_safe_stop(now, "obstacle_too_wide_no_passage")
            self._avoid_turn_dir = direction
            self._start_avoid_phase(ObstacleAvoidPhase.TURN_OUT, world)

        if self._avoid_phase is None:
            return self._build_decision(now, self.current_state, "waiting_for_obstacle", 0.0, 0.0)

        # Delegate to the active sequence handler (already running)
        result = self._handle_active_avoid_sequence(now, world)
        if result is not None:
            return result

        return self._build_decision(now, self.current_state, "obstacle_avoid_mode", 0.0, 0.0)

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def step(
        self,
        world: WorldState,
        requested_mode: BehaviorState,
        heartbeat_ok: bool,
    ) -> PlannerDecision:
        if self.cfg.DEBUG:
            debug_msg = f"DEBUG: Planner step — mode={requested_mode} phase={self._avoid_phase} obstacle={world.obstacle_ahead} lane={world.lane_detected}"
            print(debug_msg)

        now = time.monotonic()

        if not heartbeat_ok or world.stale:
            return self._decision_safe_stop(now, "heartbeat_timeout_or_stale")

        if requested_mode == BehaviorState.MANUAL:
            return self._decision_manual(now)

        if requested_mode not in (BehaviorState.LINE_FOLLOW, BehaviorState.OBSTACLE_AVOID):
            return self._decision_idle(now)

        # If an avoidance sequence is already running, service it first
        # regardless of the requested mode (avoid interrupted mid-manoeuvre).
        active_avoid = self._handle_active_avoid_sequence(now, world)
        if active_avoid is not None:
            return active_avoid

        if requested_mode == BehaviorState.LINE_FOLLOW:
            return self._handle_line_follow_mode(now, world)

        if requested_mode == BehaviorState.OBSTACLE_AVOID:
            return self._handle_obstacle_avoid_mode(now, world)

        return self._decision_idle(now)