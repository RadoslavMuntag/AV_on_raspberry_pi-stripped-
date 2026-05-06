from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import AsyncGenerator

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.concurrency import asynccontextmanager
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .schemas import (
    AcquireControllerRequest,
    ApiMessage,
    ConfigUpdateRequest,
    DriveCommand,
    HeartbeatRequest,
    LedCommand,
    ServoCommand,
    SetModeRequest,
)
from .services.hardware import VehicleHardware
from .services.runtime import RuntimeManager
from .services.state import StateStore

class NoCacheStaticFiles(StaticFiles):
    def is_not_modified(self, response_headers, request_headers) -> bool:
        return False

    def file_response(self, *args, **kwargs):
        response = super().file_response(*args, **kwargs)
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response

# Note: Startup/shutdown events are now handled in the lifespan context manager below, 
# to ensure proper async handling and avoid potential issues with async tasks during startup/shutdown.

state_store = StateStore()
hardware = VehicleHardware()
runtime = RuntimeManager(state_store=state_store, hardware=hardware)

@asynccontextmanager
async def lifespan(app: FastAPI):
    await runtime.start()
    yield
    await runtime.stop()

app = FastAPI(title="Tank AV Backend", version="0.1.0", lifespan=lifespan)

# Serve files from static directory
static_root = Path(__file__).parent / "static"
app.mount("/static", NoCacheStaticFiles(directory=static_root), name="static")


#---------------------
#--- API Endpoints ---
#---------------------

@app.get("/", response_class=FileResponse)
async def root() -> FileResponse:
    return FileResponse(static_root / "index.html")


@app.get("/api/state")
async def get_state() -> dict:
    return state_store.snapshot()


@app.post("/api/controller/acquire", response_model=ApiMessage)
async def acquire_controller(payload: AcquireControllerRequest) -> ApiMessage:
    if not runtime.acquire_controller(payload.client_id):
        raise HTTPException(status_code=409, detail="Controller already locked by another client")
    return ApiMessage(message="controller acquired")


@app.post("/api/controller/release", response_model=ApiMessage)
async def release_controller(payload: AcquireControllerRequest) -> ApiMessage:
    runtime.release_controller(payload.client_id)
    return ApiMessage(message="controller released")


@app.post("/api/controller/heartbeat", response_model=ApiMessage)
async def controller_heartbeat(payload: HeartbeatRequest) -> ApiMessage:
    if not runtime.heartbeat(payload.client_id):
        raise HTTPException(status_code=409, detail="client is not current controller")
    return ApiMessage(message="heartbeat accepted")


@app.post("/api/control/drive", response_model=ApiMessage)
async def drive(client_id: str, payload: DriveCommand) -> ApiMessage:
    if not runtime.drive(client_id=client_id, left=payload.left, right=payload.right):
        raise HTTPException(status_code=409, detail="drive rejected")
    return ApiMessage(message="drive command applied")


@app.post("/api/control/servo", response_model=ApiMessage)
async def servo(client_id: str, payload: ServoCommand) -> ApiMessage:
    if not runtime.heartbeat(client_id):
        raise HTTPException(status_code=409, detail="client is not current controller")
    hardware.set_servo(payload.index, payload.angle)
    return ApiMessage(message="servo command applied")


@app.post("/api/control/led", response_model=ApiMessage)
async def led(payload: LedCommand) -> ApiMessage:
    hardware.set_led(payload.mode, payload.r, payload.g, payload.b, payload.index)
    return ApiMessage(message="led command applied")


@app.post("/api/mode", response_model=ApiMessage)
async def set_mode(payload: SetModeRequest) -> ApiMessage:
    runtime.set_mode(payload.mode)
    return ApiMessage(message=f"mode set to {payload.mode}")


@app.post("/api/config", response_model=ApiMessage)
async def update_config(payload: ConfigUpdateRequest) -> ApiMessage:
    state_store.update_config(**payload.model_dump())
    return ApiMessage(message="config updated")

@app.post("/api/controller/dualsense/connect", response_model=ApiMessage)
async def dualsense_connect() -> ApiMessage:
    if not runtime.connect_dualsense():
        raise HTTPException(status_code=404, detail="DualSense controller not found")
    return ApiMessage(message="dualsense connected")

@app.post("/api/config/pipeline/reload", response_model=ApiMessage)
async def reload_pipeline_config() -> ApiMessage:
    if not runtime.reload_pipeline_config():
        raise HTTPException(status_code=500, detail="Failed to reload pipeline config")
    return ApiMessage(message="pipeline config reloaded")


@app.get("/video/mjpeg")
async def mjpeg_stream() -> StreamingResponse:
    async def stream() -> AsyncGenerator[bytes, None]:
        boundary = b"--frame\r\n"
        while True:
            frame = await asyncio.to_thread(hardware.get_debug_jpeg_frame)
            if not frame:
                await asyncio.sleep(0.05)
                continue
            yield boundary
            yield b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    return StreamingResponse(stream(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws/telemetry")
async def ws_telemetry(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_text(json.dumps(state_store.snapshot()))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        return
    
@app.websocket("/ws/pipeline")
async def ws_pipeline(websocket: WebSocket) -> None:
    """
    Query params:
      - frame: all|perception|world|planner|manual|control (default: all)
    """
    frame = websocket.query_params.get("frame", "all")
    print(f"New pipeline websocket connection with frame={frame}")
    valid_frames = {"all", "perception", "world", "planner", "manual", "control"}

    await websocket.accept()
    if frame not in valid_frames:
        await websocket.close(code=1008, reason="invalid frame")
        return

    try:
        while True:
            payload = state_store.pipeline_snapshot(frame=frame)
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        return
    except Exception as e:
        print(f"Error in ws_pipeline: {e}")
