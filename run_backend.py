
from __future__ import annotations

import uvicorn


if __name__ == "__main__":
    uvicorn.run("backend.app:app", host="0.0.0.0", port=8080, reload=True, timeout_graceful_shutdown=0)