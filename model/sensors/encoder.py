from gpiozero import DigitalInputDevice

class MotorEncoder:
    def __init__(
        self,
        signal_pin: int = 16,
        ticks_per_revolution: int = 10,
        pull_up: bool = True,
        bounce_time: float = 0.001, # 1 ms debounce time to prevent false ticks
    ) -> None:
        self.encoder: DigitalInputDevice = DigitalInputDevice(
            signal_pin,
            pull_up=pull_up,
            bounce_time=bounce_time,
        )
        self.ticks_per_revolution: int = ticks_per_revolution
        self.current_ticks: int = 0
        self.total_ticks: int = 0
        self._last_speed = 0.0
        self._dt_accumulation = 0.0
        self._zero_ticks_counter = 0


        self.encoder.when_activated = self._update_ticks

    def _update_ticks(self) -> None:
        self.current_ticks += 1
        self.total_ticks += 1

    def reset(self) -> None:
        self.current_ticks = 0

    def get_distance(self, wheel_circumference: float) -> float:
        """Calculate distance traveled based on current ticks and wheel circumference.
            USE BEFORE CALLING reset() OR get_speed() TO AVOID LOSING TICK COUNT
        """
        revolutions = self.current_ticks / self.ticks_per_revolution
        return revolutions * wheel_circumference

    def get_total_distance(self, wheel_circumference: float) -> float:
        """Return the cumulative distance traveled since the encoder was created."""
        revolutions = self.total_ticks / self.ticks_per_revolution
        return revolutions * wheel_circumference

    def get_speed(self, wheel_circumference: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be > 0")
        
        distance = self.get_distance(wheel_circumference)

        self._dt_accumulation += dt
        if self.current_ticks == 0:
            self._zero_ticks_counter += 1
            if self._zero_ticks_counter > 10:  # Adjust threshold as needed
                self._dt_accumulation = 0.0 
                self._last_speed = 0.0
            return self._last_speed

        self._zero_ticks_counter = 0
        self.reset()  # Reset ticks after calculating speed for the interval

        speed = distance / self._dt_accumulation
        self._dt_accumulation = 0.0  # Reset time accumulation after calculating speed
        self._last_speed = speed
        return speed

    def close(self) -> None:
        self.encoder.close()