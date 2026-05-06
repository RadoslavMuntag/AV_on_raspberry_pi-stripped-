
from __future__ import annotations

import asyncio
from _asyncio import Task

import time
import threading
from _thread import lock

from .hardware import VehicleHardware
from .state import StateStore
from ..pipeline.pipeline import ModularPipeline
from ..contracts import BehaviorState, ManualCommand
from ..misc.dualsense.dualsense import DualSense

class RuntimeManager:
    def __init__(self, state_store: StateStore, hardware: VehicleHardware) -> None:
        self.state_store : StateStore = state_store
        self.hardware : VehicleHardware = hardware
        self.pipeline: ModularPipeline = ModularPipeline()

        self._manual_cmd: ManualCommand = ManualCommand()
        self._manual_cmd_lock: lock = threading.Lock()

        self._running : bool = False
        self._telemetry_task: Task[None] | None= None
        self._control_task: Task[None] | None = None
        
        self._dualsense: DualSense | None = None

        # Control-loop FPS stats
        self.control_loop_fps: float = 0.0
        self._control_fps_frames: int = 0
        self._control_fps_t0: float = time.perf_counter()

    async def start(self) -> None:
        self.hardware.start()
        self.state_store.update_state(
        hardware_ready=self.hardware.ready,
            hardware_error=self.hardware.error,
        )
        if self.hardware.ready:
            try:
                self.hardware.start_camera_stream()
                self.state_store.update_state(camera_streaming=True)
            except Exception as exc:
                self.state_store.update_state(camera_streaming=False, hardware_error=str(exc))
        self._running = True
        self._telemetry_task = asyncio.create_task(self._telemetry_loop()) 
        self._control_task = asyncio.create_task(self._control_loop())

        _ = self.connect_dualsense()

    
    async def stop(self) -> None:
        self._running = False

        tasks = []

        if self._telemetry_task:
            self._telemetry_task.cancel()
            tasks.append(self._telemetry_task)
        if self._control_task:
            self._control_task.cancel()
            tasks.append(self._control_task)

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

        self.hardware.stop()
        self.state_store.update_state(
            camera_streaming=False,
            hardware_ready=False,
            controller_id=None,
            left_motor=0,
            right_motor=0,
        )

    def connect_dualsense(self) -> bool:
        if not self._dualsense:
            self._dualsense = DualSense(self.state_store, self.submit_manual_command)
        if self._dualsense.is_connected():
            print("DualSense already connected.")
            return True
        self._dualsense.init()
        if self._dualsense.is_connected():
            print("DualSense controller connected.")
            return True
        else:
            print("DualSense controller not found. Continuing without controller.")
            return False

    def acquire_controller(self, client_id: str) -> bool:
        snap = self.state_store.snapshot()["state"]
        current = snap["controller_id"]
        if current and current != client_id:
            return False
        self.state_store.update_state(controller_id=client_id, controller_last_seen=time.time())
        return True

    def release_controller(self, client_id: str) -> None:
        snap = self.state_store.snapshot()["state"]
        current = snap["controller_id"]
        if current == client_id:
            self.hardware.stop_motors()
            self.state_store.update_state(controller_id=None, left_motor=0, right_motor=0)

    def heartbeat(self, client_id: str) -> bool:
        snap = self.state_store.snapshot()["state"]
        if snap["controller_id"] != client_id:
            return False
        self.state_store.update_state(controller_last_seen=time.time())
        return True

    def set_mode(self, mode: BehaviorState) -> None:
        if mode == BehaviorState.SAFE_STOP:
            self.hardware.stop_motors()
            self.state_store.update_state(left_motor=0, right_motor=0, e_stop=True)

        elif mode in BehaviorState.__members__:
            if mode != BehaviorState.SAFE_STOP:
                self.state_store.update_state(e_stop=False)
        self.state_store.update_state(mode=mode)

    def clear_e_stop(self) -> None:
        self.state_store.update_state(e_stop=False)

    def reload_pipeline_config(self, json_path: str = "backend/config/pipeline.json") -> bool:
        try:
            _ = self.pipeline.load_config_from_json(json_path)
            print("Pipeline config reloaded successfully.")
            return True
        except Exception as e:
            print(f"Error loading pipeline config: {e}")
            return False

    def drive(self, client_id: str, left: int, right: int) -> bool:
        snap = self.state_store.snapshot()
        state = snap["state"]
        cfg = snap["config"]
        
        if state["controller_id"] != client_id or state["e_stop"]:
            return False

        max_speed = cfg["max_motor_speed"]
        left = max(-max_speed, min(max_speed, left))
        right = max(-max_speed, min(max_speed, right))


        # Convert wheel command -> normalized manual command for pipeline controller
        throttle = (left + right) / (2.0 * max_speed)
        steer = (right - left) / (2.0 * max_speed)

        with self._manual_cmd_lock:
            self._manual_cmd.throttle = max(-1.0, min(1.0, throttle))
            self._manual_cmd.steer = max(-1.0, min(1.0, steer))
            self._manual_cmd.active = True

        self.state_store.update_state(
            controller_last_seen=time.time(),
            left_motor=left,
            right_motor=right,
        )
        return True
    
    def set_car_mode(self, mode: BehaviorState) -> None:
        """Called by DualSense handler to switch between manual and autonomous mode."""
        if mode == BehaviorState.MANUAL:
            self.state_store.update_state(e_stop=False)
        self.state_store.update_state(requested_mode=mode)
    
    def submit_manual_command(self, cmd: ManualCommand) -> None:
        """Called by DualSense handler to submit manual driving commands."""
        with self._manual_cmd_lock:
            # copy to avoid external mutation/races
            self._manual_cmd = ManualCommand(
                throttle=cmd.throttle,
                steer=cmd.steer,
                active=cmd.active,
            )

    async def _telemetry_loop(self) -> None:
        try:
            while self._running:
                self.state_store.update_state(
                    ultrasonic_cm=self.hardware.read_ultrasonic(),
                    #infrared_value=self.hardware.read_infrared(),
                    hardware_ready=self.hardware.ready,
                    hardware_error=self.hardware.error,
                )
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass

    async def _control_loop(self) -> None:
        try:
            last_t = time.perf_counter()
            while self._running:
                now = time.perf_counter()
                dt = now - last_t
                last_t = now

                snap = self.state_store.snapshot()
                state = snap["state"]
                cfg = snap["config"]
                loop_delay = 1.0 / cfg["control_loop_hz"]

                timed_out = self.state_store.should_timeout_controller()
                if timed_out:
                    self.state_store.update_state(
                        controller_id=None,
                        mode=BehaviorState.SAFE_STOP,
                        e_stop=True,
                    )

                requested_mode = state["mode"]
                heartbeat_ok = True
                if requested_mode == BehaviorState.MANUAL:
                    heartbeat_ok = (state["controller_id"] is not None) and (not timed_out)

                with self._manual_cmd_lock:
                    cmd_for_tick = ManualCommand(
                        throttle=self._manual_cmd.throttle,
                        steer=self._manual_cmd.steer,
                        active=(requested_mode == BehaviorState.MANUAL and heartbeat_ok),
                    )

                pipe = self.pipeline.tick(
                    hardware=self.hardware,
                    requested_mode=requested_mode,
                    heartbeat_ok=heartbeat_ok and not state["e_stop"],
                    manual_cmd=cmd_for_tick,
                    dt=dt,
                )

                self.state_store.set_pipeline_snapshot(
                    perception=pipe.perception, 
                    world=pipe.world, 
                    decision=pipe.decision, 
                    control=pipe.control
                    )

                self.state_store.set_manual_command(cmd_for_tick)

                requested_mode = self.state_store.snapshot()["state"]["requested_mode"]
                temp_mode = requested_mode if requested_mode is not None else pipe.decision.state.value

                self.state_store.update_state(
                    mode=temp_mode,
                    requested_mode=None,
                    left_motor=pipe.control.left_pwm,
                    right_motor=pipe.control.right_pwm,
                    ultrasonic_cm=pipe.perception.ultrasonic_cm,
                    hardware_ready=self.hardware.ready,
                    hardware_error=self.hardware.error,
                )

                self._control_fps_frames += 1
                elapsed = now - self._control_fps_t0
                if elapsed >= 1.0:
                    self.control_loop_fps = self._control_fps_frames / elapsed
                    self.state_store.update_state(fps=self.control_loop_fps)
                    self._control_fps_frames = 0
                    self._control_fps_t0 = now

                await asyncio.sleep(loop_delay)
        except asyncio.CancelledError:
            print("Control loop cancelled, stopping motors.")
            pass
        except Exception as exc:
            print(f"Exception in control loop: {exc}")
            self.hardware.stop_motors()
            self.state_store.update_state(left_motor=0, right_motor=0, runtime_error=str(exc))
