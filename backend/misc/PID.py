from __future__ import annotations

from dataclasses import dataclass


def _clamp(value: float, lower: float | None, upper: float | None) -> float:
	if lower is not None and value < lower:
		return lower
	if upper is not None and value > upper:
		return upper
	return value


@dataclass
class PIDTerms:
	p: float = 0.0
	i: float = 0.0
	d: float = 0.0

	@property
	def total(self) -> float:
		return self.p + self.i + self.d


class SpeedPIDController:
	"""Simple PID controller for wheel/motor speed regulation.

	Usage:
		pid = SpeedPIDController(kp=1.2, ki=0.2, kd=0.05)
		pid.set_setpoint(20.0)  # target speed (unit chosen by caller)
		control = pid.update(measured_speed=current_speed, dt=0.02)  # control in [-1.0, 1.0]
	"""

	def __init__(
		self,
		kp: float,
		ki: float,
		kd: float,
		setpoint: float = 0.0,
		output_limits: tuple[float | None, float | None] = (-1.0, 1.0),
		integral_limits: tuple[float | None, float | None] = (None, None),
		derivative_on_measurement: bool = False,
	) -> None:
		self.kp: float = float(kp)
		self.ki: float = float(ki)
		self.kd: float = float(kd)

		self.setpoint: float = float(setpoint)
		self.output_limits: tuple[float | None, float | None] = output_limits
		self.integral_limits: tuple[float | None, float | None] = integral_limits
		self.derivative_on_measurement: bool = derivative_on_measurement

		self._integral: float = 0.0
		self._prev_error: float | None = None
		self._prev_measurement: float | None = None
		self._terms: PIDTerms = PIDTerms()

	@property
	def terms(self) -> PIDTerms:
		return self._terms

	def set_setpoint(self, setpoint: float) -> None:
		self.setpoint = float(setpoint)

	def tune(self, kp: float | None = None, ki: float | None = None, kd: float | None = None) -> None:
		if kp is not None:
			self.kp = float(kp)
		if ki is not None:
			self.ki = float(ki)
		if kd is not None:
			self.kd = float(kd)

	def reset(self) -> None:
		self._integral = 0.0
		self._prev_error = None
		self._prev_measurement = None
		self._terms = PIDTerms()

	def update(self, measured_speed: float, dt: float) -> float:
		if dt <= 0.0:
			raise ValueError("dt must be > 0")

		error = self.setpoint - float(measured_speed)

		p_term = self.kp * error

		self._integral += error * dt
		int_lo, int_hi = self.integral_limits
		self._integral = _clamp(self._integral, int_lo, int_hi)
		i_term = self.ki * self._integral

		if self.derivative_on_measurement:
			if self._prev_measurement is None:
				derivative = 0.0
			else:
				derivative = -(float(measured_speed) - self._prev_measurement) / dt
		else:
			if self._prev_error is None:
				derivative = 0.0
			else:
				derivative = (error - self._prev_error) / dt
		d_term = self.kd * derivative

		raw_output = p_term + i_term + d_term
		out_lo, out_hi = self.output_limits
		output = _clamp(raw_output, out_lo, out_hi)

		if self.ki != 0.0 and output != raw_output:
			self._integral = (output - p_term - d_term) / self.ki
			self._integral = _clamp(self._integral, int_lo, int_hi)
			i_term = self.ki * self._integral

		self._terms = PIDTerms(p=p_term, i=i_term, d=d_term)
		self._prev_error = error
		self._prev_measurement = float(measured_speed)

		return output
