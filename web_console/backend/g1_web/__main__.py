import uvicorn

from .app import create_app
from .config import Settings


if __name__ == "__main__":
    settings = Settings.from_env()
    # One worker owns the controller lease and mock mission. Multiple workers require shared state.
    # Keep HTTP connections beyond Safari's idle reuse window (about 29 seconds).
    uvicorn.run(create_app(settings), host=settings.host, port=settings.port,
                workers=1, ws_max_size=4096, access_log=False, timeout_keep_alive=60)
