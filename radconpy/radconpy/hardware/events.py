from enum import Enum
from typing import Callable

from radconpy.api import Box, Event


class ConnectionErrorReason(Enum):
    ConnectionAborted = "ConnectionAborted"
    ConnectionAttemptFailed = "ConnectionAttemptFailed"
    ConnectionLost = "ConnectionLost"
    Other = "Other"


OnSerialDeviceConnectingDelegate = Callable[[Box[bool]], None]
OnSerialDeviceConnectedDelegate = Callable[[], None]
OnSerialDeviceConnectionErrorDelegate = Callable[[ConnectionErrorReason], None]
OnSerialDeviceDisconnectedDelegate = Callable[[], None]


class OnSerialDeviceConnecting(Event[OnSerialDeviceConnectingDelegate]):
    def broadcast(self, should_allow: Box[bool]):
        return super().broadcast(should_allow)


class OnSerialDeviceConnected(Event[OnSerialDeviceConnectedDelegate]):
    def broadcast(self):
        return super().broadcast()


class OnSerialDeviceConnectionError(
    Event[OnSerialDeviceConnectionErrorDelegate]
):
    def broadcast(self, reason: ConnectionErrorReason):
        return super().broadcast(reason)


class OnSerialDeviceDisconnected(Event[OnSerialDeviceDisconnectedDelegate]):
    def broadcast(self):
        return super().broadcast()
