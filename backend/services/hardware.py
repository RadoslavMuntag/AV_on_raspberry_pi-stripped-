from __future__ import annotations

from cv2.typing import MatLike
import math

from model.sensors.camera import Camera

from model.sensors.ultrasonic import Ultrasonic
from model.actuators.motor import tankMotor
from model.actuators.servo import Servo
from model.sensors.infrared import Infrared
from model.sensors.encoder import MotorEncoder

from model.misc.led import Led
from ..misc.usb_camera import USBCamera

PI_CAM = True
USB_CAM = False

REVERSED_MOTORS_DIRECTION = True  # Set to True if motors are wired in reverse and need to be flipped in software

class Car:
    """Legacy car interface. This is used internally by VehicleHardware, 
    but is not exposed to the pipeline modules."""

    def __init__(self):
        self.servo: Servo = Servo()
        self.sonic: Ultrasonic = Ultrasonic()
        self.motor: tankMotor = tankMotor()
        self.infrared: Infrared = None
        self.left_encoder: MotorEncoder = MotorEncoder(signal_pin=19, ticks_per_revolution=10)
        self.right_encoder: MotorEncoder = MotorEncoder(signal_pin=16, ticks_per_revolution=10)


    def close(self):
        self.servo.setServoStop()
        self.sonic.close()
        self.motor.close()
        #self.infrared.close()
        self.left_encoder.close()
        self.right_encoder.close()


class VehicleHardware:
    """A wrapper around the old hardware interface. 
    The main purpose is to isolate key hardware interactions 
    and provide a clean API for the pipeline modules.
    """

    def __init__(self) -> None:
        self._camera: Camera | None = None
        self._usb_camera : USBCamera | None = None

        self._car: Car | None = None

        self._led: Led | None = None
        self._ready: bool = False
        self._error: str | None = None 

    @property
    def ready(self) -> bool:
        return self._ready

    @property
    def error(self) -> str | None:
        return self._error

    def start(self) -> None:
        try:
            self._car = Car()
            self._led = Led()
            self._camera = Camera(stream_size=(1280, 720))
            self._usb_camera = USBCamera(stream_size=(400, 300))
            self._ready = True
            self._error = None
        except Exception as exc:
            self._ready = False
            self._error = str(exc)

    def stop(self) -> None:
        try:
            if self._camera and PI_CAM:
                try:
                    self._camera.stop_stream()
                except Exception:
                    pass
                self._camera.close()
            if self._usb_camera and USB_CAM:
                try:
                    self._usb_camera.stop_stream()
                except Exception:
                    pass
                self._usb_camera.close()
            if self._led:
                self._led.colorWipe((0, 0, 0), 10)
            if self._car:
                self.stop_motors()
                self._car.close()
        finally:
            self._car = None
            self._camera = None
            self._usb_camera = None
            self._led = None
            self._ready = False

    def start_camera_stream(self) -> None:
        if self._camera and PI_CAM:
            self._camera.start_stream()
        if self._usb_camera and USB_CAM:
            self._usb_camera.start_stream()

    def stop_camera_stream(self) -> None:
        if self._camera and PI_CAM:
            self._camera.stop_stream()
        if self._usb_camera and USB_CAM:
            self._usb_camera.stop_stream()

    def get_jpeg_frame(self) -> bytes | None:
        if self._camera and PI_CAM:
            return self._camera.get_frame()
        return None

    def get_usb_jpeg_frame(self) -> bytes | None:
        if self._usb_camera and USB_CAM:
            return self._usb_camera.get_frame()
        return None

    def set_debug_frame(self, frame: MatLike) -> None:
        if self._camera and PI_CAM:
            self._camera.set_debug_frame(frame)

    def get_debug_jpeg_frame(self) -> bytes | None:
        if self._camera and PI_CAM:
            return self._camera.get_debug_frame()
        return None

    def stop_motors(self) -> None:
        if self._car:
            self._car.motor.setMotorModel(0, 0)

    def set_motor(self, left: int, right: int) -> None:
        """Set motor speeds. Expects values in range [-4095, 4095]. Positive is forward."""
        if self._car:
            if REVERSED_MOTORS_DIRECTION:
                left, right = -left, -right
            self._car.motor.setMotorModel(left, right)

    def set_servo(self, index: int, angle: int) -> None:
        """Set servo angle. Expects index in [0, 2] and angle in [0, 180]."""
        if self._car:
            self._car.servo.setServoAngle(index, angle)

    def set_led(self, mode: str, r: int, g: int, b: int, index: int) -> None:
        """Set LED mode and color. Mode can be 'off', 'index', 'blink', 'breathing', or 'rainbow'."""
        if not self._led:
            return
        if mode == "off":
            self._led.colorWipe((0, 0, 0), 10)
        elif mode == "index":
            self._led.ledIndex(index, r, g, b)
        elif mode == "blink":
            self._led.Blink((r, g, b), 50)
            self._led.Blink((0, 0, 0), 50)
        elif mode == "breathing":
            self._led.Breathing((r, g, b))
        elif mode == "rainbow":
            self._led.rainbowCycle()

    def read_ultrasonic(self) -> float | None:
        """Returns distance in cm, or None if error."""
        if self._car:
            try:
                return float(self._car.sonic.get_distance())
            except Exception:
                return None
        return None

    # def read_infrared(self) -> int | None:
    #     """Returns raw 3-bit pattern from IR sensors, or None if error."""
    #     if self._car:
    #         try:
    #             return int(self._car.infrared.read_all_infrared())
    #         except Exception:
    #             return None
    #     return None

    def read_left_encoder(self, wheel_radius: float, dt: float) -> float | None:
        """Returns distance in cm from left motor encoder, or None if error."""
        if self._car:
            try:
                return self._car.left_encoder.get_speed(wheel_radius * 2 * math.pi, dt)
            except Exception:
                return None
        return None

    def read_right_encoder(self, wheel_radius: float, dt: float) -> float | None:
        """Returns distance in cm from right motor encoder, or None if error."""
        if self._car:
            try:
                return self._car.right_encoder.get_speed(wheel_radius * 2 * math.pi, dt)
            except Exception:
                return None
        return None

    def read_left_encoder_distance(self, wheel_radius: float) -> float | None:
        """Returns cumulative distance in cm from the left motor encoder, or None if error."""
        if self._car:
            try:
                return self._car.left_encoder.get_total_distance(wheel_radius * 2 * math.pi)
            except Exception:
                return None
        return None
    
    def read_right_encoder_distance(self, wheel_radius: float) -> float | None:
        """Returns cumulative distance in cm from the right motor encoder, or None if error."""
        if self._car:
            try:
                return self._car.right_encoder.get_total_distance(wheel_radius * 2 * math.pi)
            except Exception:
                return None
        return None