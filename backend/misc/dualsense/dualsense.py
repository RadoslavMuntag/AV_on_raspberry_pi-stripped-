from evdev.device import InputDevice

import evdev
import threading
from typing import Callable

from ...services.state import StateStore
from ...contracts import BehaviorState, ManualCommand

class DualSense:
    def __init__(self, state_store: StateStore, submit_manual_cmd_callback: Callable[[ManualCommand], None]) -> None:
        self.state_store: StateStore = state_store
        self.device: InputDevice[str] | None = None
        self.connected: bool = False
        self.thread: threading.Thread | None = None

        # Callback to submit manual commands to the runtime manager
        self.submit_manual_cmd_callback: Callable[[ManualCommand], None] = submit_manual_cmd_callback

        # Servo angles for incremental control
        self.servo_angles: list[int] = [90, 140, 90]  # Initial angles for servos 0, 1, 2
        self.servo_step: int = 5  # Degrees to change per D-pad press

    def is_connected(self) -> bool:
        return self.connected

    def _find_controller(self) -> InputDevice[str] | None:
        """Find the DualSense controller device"""
        devices: list[InputDevice[str]] = [evdev.InputDevice(path) for path in evdev.list_devices()]
        for device in devices:
            if "DualSense" in device.name and "Motion" not in device.name and "Touchpad" not in device.name:
                return device
        return None

    def init(self) -> bool:
        try:
            self.device = self._find_controller()
            if not self.device:
                print("DualSense controller not found")
                return False

            print(f"DualSense connected: {self.device.name}")
            self.connected = True
            self.state_store.update_state(dualsense_connected=True)

            self.thread = threading.Thread(target=self._read_loop)
            self.thread.daemon = True
            self.thread.start()
            return True

        except Exception as e:
            print(f"Failed to initialize DualSense device: {e}")
            return False

    def _read_loop(self):
        """Main loop to read controller inputs"""
        try:
            for event in self.device.read_loop():
                if not self.connected:
                    break

                if event.type == evdev.ecodes.EV_ABS:
                    # Handle joystick and trigger inputs
                    if event.code == evdev.ecodes.ABS_X:
                        self._handle_joystick(event.value, None)  # X axis
                    elif event.code == evdev.ecodes.ABS_Y:
                        self._handle_joystick(None, event.value)  # Y axis
                    elif event.code == evdev.ecodes.ABS_Z:
                        self._handle_l2(event.value)
                    elif event.code == evdev.ecodes.ABS_RZ:
                        self._handle_r2(event.value)
                    # elif event.code == evdev.ecodes.ABS_HAT0X:
                    #     self._handle_dpad_x(event.value)
                    # elif event.code == evdev.ecodes.ABS_HAT0Y:
                    #     self._handle_dpad_y(event.value)
                elif event.type == evdev.ecodes.EV_KEY:
                    # Handle button presses
                    if event.value == 1:  # Button pressed (not released)
                        if event.code == evdev.ecodes.BTN_SOUTH:  # X button
                            self.state_store.update_state(requested_mode=BehaviorState.MANUAL)
                        elif event.code == evdev.ecodes.BTN_WEST:  # Square button
                            self.state_store.update_state(requested_mode=BehaviorState.OBSTACLE_AVOID)
                        elif event.code == evdev.ecodes.BTN_EAST:  # Circle button
                            self.state_store.update_state(requested_mode=BehaviorState.SAFE_STOP)
                        elif event.code == evdev.ecodes.BTN_NORTH:  # Triangle button
                            self.state_store.update_state(requested_mode=BehaviorState.LINE_FOLLOW)

                        print(f"Button event: code={event.code}, value={event.value}")


            self.close()
            print("DualSense controller disconnected")

        except OSError:
            print("DualSense controller disconnected")
            self.close()
        except Exception as e:
            print(f"Error reading controller: {e}")
            self.close()

    def _handle_joystick(self, x_value, y_value) -> None:
        """Handle left joystick input"""
        if x_value is not None:
            self.last_x = x_value
        if y_value is not None:
            self.last_y = y_value

        if hasattr(self, 'last_x') and hasattr(self, 'last_y'):
            self.joystick(self.last_x, self.last_y)

    def _handle_l2(self, value):
        """Handle L2 trigger"""
        self.on_l2_value_changed(value)

    def _handle_r2(self, value):
        """Handle R2 trigger"""
        self.on_r2_value_changed(value)

    def joystick(self, stateX, stateY):

        # Convert from evdev range (0-255, center 128) to normalized (-1 to 1)
        center = 128.0
        max_range = 128.0

        # Normalize to -1 to 1
        linear_norm = -(stateY - center) / max_range  # Y is inverted
        angular_norm = -(stateX - center) / max_range

        if abs(linear_norm) < 0.01 and abs(angular_norm) < 0.01:
            return
    
        self.submit_manual_cmd_callback(ManualCommand(throttle=linear_norm, steer=angular_norm, active=True))     

    def on_l2_value_changed(self, value):
        # Scale trigger value (0-255)
        # L2 is backward, so negative values
        speed = (value / 255.0)
        self.on_btn_BackWard(speed)

    def on_r2_value_changed(self, value):
        # Scale trigger value (0-255)
        # R2 is forward, so positive values
        speed = (value / 255.0)
        self.on_btn_ForWard(speed)

    # def _handle_dpad_x(self, value):
    #     """Handle D-pad left/right for servo 1 (horizontal movement)"""
    #     if value == -1:  # Left pressed
    #         self.servo_angles[1] = min(150, self.servo_angles[1] + self.servo_step)
    #         self.hardware.set_servo(1, self.servo_angles[1])
    #         print(f"Servo 1 (horizontal): {self.servo_angles[1]}°")
    #     elif value == 1:  # Right pressed
    #         self.servo_angles[1] = max(90, self.servo_angles[1] - self.servo_step)
    #         self.hardware.set_servo(1, self.servo_angles[1])
    #         print(f"Servo 1 (horizontal): {self.servo_angles[1]}°")

    # def _handle_dpad_y(self, value):
    #     """Handle D-pad up/down for servo 0 (vertical movement)"""
    #     if value == -1:  # Up pressed
    #         self.servo_angles[0] = min(150, self.servo_angles[0] + self.servo_step)
    #         self.hardware.set_servo(0, self.servo_angles[0])
    #         print(f"Servo 0 (vertical): {self.servo_angles[0]}°")
    #     elif value == 1:  # Down pressed
    #         self.servo_angles[0] = max(90, self.servo_angles[0] - self.servo_step)
    #         self.hardware.set_servo(0, self.servo_angles[0])
    #         print(f"Servo 0 (vertical): {self.servo_angles[0]}°")

    def on_btn_ForWard(self, value):
        self.submit_manual_cmd_callback(ManualCommand(throttle=value, steer=0.0, active=True))

    def on_btn_BackWard(self, value):
        self.submit_manual_cmd_callback(ManualCommand(throttle=-value, steer=0.0, active=True))

    def close(self):
        self.connected = False
        self.state_store.update_state(dualsense_connected=False)
        if self.device:
            self.device.close()
        if self.thread and self.thread.is_alive() and self.thread != threading.current_thread():
            self.thread.join(timeout=1.0)