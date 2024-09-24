from datetime import datetime
from typing import Callable

from .api import Event

OnRadConConnectedDelegate = Callable[[], None]
OnRadConDisconnectedDelegate = Callable[[], None]
OnRadConDataDelegate = Callable[[datetime, float, int], None]


class OnRadConConnected(Event[OnRadConConnectedDelegate]):
    def broadcast(self) -> None:
        return super().broadcast()


class OnRadConDisconnected(Event[OnRadConDisconnectedDelegate]):
    def broadcast(self) -> None:
        return super().broadcast()


class OnRadConData(Event[OnRadConDataDelegate]):
    def broadcast(self, timestamp: datetime, hardware_timestamp: float, pulse_length: int) -> None:
        return super().broadcast(timestamp, hardware_timestamp, pulse_length)
