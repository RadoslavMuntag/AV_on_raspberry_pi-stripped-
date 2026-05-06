import evdev
import threading


from model.car import Car

class DualSense:
    def __init__(self, car: Car, mode_callback=None):
        self.car: Car  = car
        self.mode_callback = mode_callback # Callback to notify mode changes (manual, ultrasonic, infrared)
        self.device: evdev.InputDevice | None = None
        self.running = False
        self.thread = None
        self.manual_mode = True  # Start in manual mode
        # Servo angles for incremental control
        self.servo_angles = [90, 140, 90]  # Initial angles for servos 0, 1, 2
        self.servo_step = 5  # Degrees to change per D-pad press

    def _find_controller(self) -> evdev.InputDevice | None:
        """Find the DualSense controller device"""
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
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
            self.running = True
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
                if not self.running:
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
                    elif event.code == evdev.ecodes.ABS_HAT0X:
                        self._handle_dpad_x(event.value)
                    elif event.code == evdev.ecodes.ABS_HAT0Y:
                        self._handle_dpad_y(event.value)
                elif event.type == evdev.ecodes.EV_KEY:
                    # Handle button presses
                    if event.value == 1:  # Button pressed (not released)
                        if event.code == evdev.ecodes.BTN_SOUTH:  # X button
                            if self.mode_callback:
                                self.mode_callback(1)  # Manual mode
                        elif event.code == evdev.ecodes.BTN_WEST:  # Square button
                            if self.mode_callback:
                                self.mode_callback(2)  # Ultrasonic mode
                        elif event.code == evdev.ecodes.BTN_EAST:  # Circle button
                            if self.mode_callback:
                                self.mode_callback(3)  # Infrared mode

        except OSError:
            pass  # Expected when device is closed
        except Exception as e:
            print(f"Error reading controller: {e}")

    def _handle_joystick(self, x_value, y_value):
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
        if not self.manual_mode:
            return  # Only control motors in manual mode

        # Convert from evdev range (0-255, center 128) to normalized (-1 to 1)
        center = 128.0
        max_range = 128.0

        # Normalize to -1 to 1
        linear_norm = -(stateY - center) / max_range  # Y is inverted
        angular_norm = (stateX - center) / max_range

        # Scale to motor speeds
        max_speed = 4095
        linear_speed = linear_norm * max_speed
        angular_speed = angular_norm * max_speed

        left_motor = int(linear_speed + angular_speed)
        right_motor = int(linear_speed - angular_speed)

        # Clamp to motor limits
        left_motor = max(-max_speed, min(max_speed, left_motor))
        right_motor = max(-max_speed, min(max_speed, right_motor))

        # Directly control motors
        self.car.motor.setMotorModel(left_motor, right_motor)

    def on_l2_value_changed(self, value):
        # Scale trigger value (0-255) to motor speed (-4095 to 4095)
        # L2 is backward, so negative values
        speed = int((value / 255.0) * 4095)
        self.on_btn_BackWard(speed)

    def on_r2_value_changed(self, value):
        # Scale trigger value (0-255) to motor speed (0 to 4095)
        # R2 is forward, so positive values
        speed = int((value / 255.0) * 4095)
        self.on_btn_ForWard(speed)

    def _handle_dpad_x(self, value):
        """Handle D-pad left/right for servo 1 (horizontal movement)"""
        if value == -1:  # Left pressed
            self.servo_angles[1] = min(150, self.servo_angles[1] + self.servo_step)
            self.car.servo.setServoAngle(1, self.servo_angles[1])
            print(f"Servo 1 (horizontal): {self.servo_angles[1]}°")
        elif value == 1:  # Right pressed
            self.servo_angles[1] = max(90, self.servo_angles[1] - self.servo_step)
            self.car.servo.setServoAngle(1, self.servo_angles[1])
            print(f"Servo 1 (horizontal): {self.servo_angles[1]}°")

    def _handle_dpad_y(self, value):
        """Handle D-pad up/down for servo 0 (vertical movement)"""
        if value == -1:  # Up pressed
            self.servo_angles[0] = min(150, self.servo_angles[0] + self.servo_step)
            self.car.servo.setServoAngle(0, self.servo_angles[0])
            print(f"Servo 0 (vertical): {self.servo_angles[0]}°")
        elif value == 1:  # Down pressed
            self.servo_angles[0] = max(90, self.servo_angles[0] - self.servo_step)
            self.car.servo.setServoAngle(0, self.servo_angles[0])
            print(f"Servo 0 (vertical): {self.servo_angles[0]}°")

    def on_btn_ForWard(self, value):
        self.car.motor.setMotorModel(value, value)

    def on_btn_Turn_Left(self, value):
        self.car.motor.setMotorModel(value, -value)

    def on_btn_BackWard(self, value):
        self.car.motor.setMotorModel(-value, -value)

    def set_manual_mode(self, manual):
        self.manual_mode = manual

    def close(self):
        self.running = False
        if self.device:
            self.device.close()
        if self.thread and self.thread.is_alive() and self.thread != threading.current_thread():
            self.thread.join(timeout=1.0)