from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field

from .contracts import BehaviorState, LedMode




class AcquireControllerRequest(BaseModel):
    client_id: str = Field(..., min_length=1, max_length=128)


class DriveCommand(BaseModel):
    left: int = Field(..., ge=-4095, le=4095)
    right: int = Field(..., ge=-4095, le=4095)


class ServoCommand(BaseModel):
    index: int = Field(..., ge=0, le=2)
    angle: int = Field(..., ge=0, le=180)


class LedCommand(BaseModel):
    mode: LedMode = LedMode.OFF
    r: int = Field(0, ge=0, le=255)
    g: int = Field(0, ge=0, le=255)
    b: int = Field(0, ge=0, le=255)
    index: int = Field(0, ge=0, le=15)


class SetModeRequest(BaseModel):
    mode: BehaviorState


class HeartbeatRequest(BaseModel):
    client_id: str = Field(..., min_length=1, max_length=128)


class ConfigUpdateRequest(BaseModel):
    heartbeat_timeout_sec: Optional[float] = Field(None, gt=0.1, le=10.0)
    control_loop_hz: Optional[float] = Field(None, gt=1.0, le=200.0)
    max_motor_speed: Optional[int] = Field(None, ge=200, le=4095)


class ApiMessage(BaseModel):
    message: str
