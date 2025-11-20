from __future__ import annotations

import contextlib
import select
import socket
import struct
import threading
from typing import Callable

from loguru import logger
from .pilot import Pilot


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class DriverCtrlPilot(Pilot):
    """Pilot that receives control commands over UDP sockets."""

    def __init__(
        self,
        dt: float = 0.1,
        max_delta_v: float = 0.2617993877991494,
        max_acceleration: float = 4.0,
        max_deceleration: float = -6.0,
        host: str = "",
        gas_port: int = 7766,
        brake_port: int = 7767,
        steering_port: int = 7768,
    ) -> None:
        super().__init__()

        self._dt = dt
        self._max_delta_v = max_delta_v
        self._max_acceleration = max_acceleration
        self._max_deceleration = max_deceleration

        self._host = host
        self._ports = {
            "gas": gas_port,
            "brake": brake_port,
            "steering": steering_port,
        }

        self._gas = 0.0
        self._brake = 0.0
        self._steering = 0.0

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sockets: dict[socket.socket, Callable[[float], None]] = {}

        self._listener_thread = threading.Thread(
            target=self._listen_loop,
            name="DriverCtrlPilotListener",
            daemon=True,
        )
        self._listener_thread.start()

        logger.info(
            f"DriverCtrlPilot listening on gas={gas_port}, brake={brake_port}, steering={steering_port}"
        )

    def step(self, observation: dict, **kwargs) -> tuple[float, float]:  # noqa: ARG002
        with self._lock:
            gas = self._gas
            brake = self._brake
            steering = self._steering

        delta_v = self._steering - _clamp(
            steering, -self._max_delta_v, self._max_delta_v
        )
        a = (gas * self._max_acceleration) + (brake * self._max_deceleration)

        self._state_list.append([gas, brake, steering])
        self._action_list.append([delta_v, a])

        return delta_v, a

    def stop(self) -> None:
        self._stop_event.set()
        for sock in list(self._sockets.keys()):
            with contextlib.suppress(OSError):
                sock.close()
        if self._listener_thread.is_alive():
            self._listener_thread.join(timeout=1.0)

    def __del__(self) -> None:
        self.stop()

    def _listen_loop(self) -> None:
        handlers: dict[str, Callable[[float], None]] = {
            "gas": self._set_gas,
            "brake": self._set_brake,
            "steering": self._set_steering,
        }

        for channel, port in self._ports.items():
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((self._host, port))
            except OSError as exc:
                logger.error(
                    "DriverCtrlPilot failed to bind {} port {}: {}", channel, port, exc
                )
                self._stop_event.set()
                return
            self._sockets[sock] = handlers[channel]

        sockets = list(self._sockets.keys())

        while not self._stop_event.is_set():
            try:
                readable, _, _ = select.select(sockets, [], [], 0.5)
            except OSError:
                break

            for sock in readable:
                try:
                    data, _ = sock.recvfrom(64)
                except OSError:
                    continue
                value = self._parse_value(data)
                if value is None:
                    continue
                handler = self._sockets.get(sock)
                if handler is not None:
                    handler(value)

        for sock in sockets:
            with contextlib.suppress(OSError):
                sock.close()
        self._sockets.clear()

    def _set_gas(self, value: float) -> None:
        clamped = _clamp(value, 0.0, 1.0)
        with self._lock:
            self._gas = clamped

    def _set_brake(self, value: float) -> None:
        clamped = _clamp(value, 0.0, 1.0)
        with self._lock:
            self._brake = clamped

    def _set_steering(self, value: float) -> None:
        with self._lock:
            self._steering = value

    @staticmethod
    def _parse_value(data: bytes) -> float | None:
        # Try binary double first (8 bytes)
        if len(data) == 8:
            try:
                return struct.unpack("d", data)[0]
            except struct.error:
                pass
        # Fall back to text float
        try:
            text = data.decode("utf-8", "ignore").strip()
            if not text:
                return None
            return float(text)
        except ValueError:
            return None
