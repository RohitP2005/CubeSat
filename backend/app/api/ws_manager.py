"""
WebSocket connection pool with per-channel broadcast.

Each channel (e.g. "/ws/attitude", "/ws/telemetry") has an independent
set of connected clients. broadcast() serialises the payload to JSON and
delivers it to all clients on that channel; dead connections are silently
pruned so the engine loop never blocks on a broken socket.
"""

import json
import logging
from collections import defaultdict

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class WebSocketManager:
    def __init__(self) -> None:
        # channel → set of live WebSocket clients
        self._connections: dict[str, set[WebSocket]] = defaultdict(set)

    async def connect(self, ws: WebSocket, channel: str) -> None:
        await ws.accept()
        self._connections[channel].add(ws)
        logger.debug("WS connect  channel=%s  total=%d", channel, len(self._connections[channel]))

    def disconnect(self, ws: WebSocket, channel: str) -> None:
        self._connections[channel].discard(ws)
        logger.debug("WS disconnect channel=%s  total=%d", channel, len(self._connections[channel]))

    async def broadcast(self, channel: str, payload: dict) -> None:
        """
        JSON-encode payload and send to every client on channel.
        Clients that raise during send are removed from the pool.
        Takes a snapshot of the connection set first so connect/disconnect
        during broadcast does not cause iteration errors.
        """
        clients = set(self._connections.get(channel, set()))
        if not clients:
            return

        message = json.dumps(payload)
        dead: set[WebSocket] = set()

        for ws in clients:
            try:
                await ws.send_text(message)
            except Exception:
                dead.add(ws)

        if dead:
            self._connections[channel] -= dead

    def client_count(self, channel: str) -> int:
        return len(self._connections.get(channel, set()))
