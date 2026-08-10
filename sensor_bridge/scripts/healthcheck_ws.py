#!/usr/bin/env python3
"""
Docker health check for the sensor bridge: opens a WebSocket to the camera port
and closes it. Using a real WebSocket handshake avoids "opening handshake failed"
log noise from raw TCP health checks that close without sending HTTP.
"""
import asyncio
import sys

import websockets


async def check(url: str = "ws://localhost:8765", timeout: float = 3.0) -> None:
    async with asyncio.timeout(timeout):
        async with websockets.connect(url) as _:
            pass


def main() -> int:
    try:
        asyncio.run(check())
        return 0
    except Exception:
        return 1


if __name__ == "__main__":
    sys.exit(main())
