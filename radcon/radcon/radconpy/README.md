# RadConPy

Library for communicating with RadCon device made in pure Python

## Running

### Installing dependencies
```sh
poetry install
```

## Interface

### Box

Simple implementation of box for passing primitive types to functions

```py
class Box[T]:
    def __init__(self, value: T): pass

    @property
    def value(self) -> T: pass

    @value.setter
    def value(self, value: T) -> None: pass

    # Gets value of the box
    def get(self) -> T: pass

    # Sets value of the box
    def set(self, value: T) -> None: pass
```

### Event

Simple implementation of observer pattern, manages registration of listeners and broadcasting data to them.

```py 
class Event[T]:
    # Registers listener
    def add(self, handler: T): pass
    
    # Removes listener
    def remove(self, handler: T): pass

    # Clears all listeners
    def clear(self) -> None: pass

    # Broadcasts data to listeners
    def broadcast(self, *args, **kwargs): pass
```

### SerialDevice

Abstraction over serial devices, exposing events for connecting, disconnecting and errors.

```py
class ConnectionErrorReason(Enum):
    ConnectionAborted = "ConnectionAborted"
    ConnectionAttemptFailed = "ConnectionAttemptFailed"
    ConnectionLost = "ConnectionLost"
    Other = "Other"

OnSerialDeviceConnectingDelegate = Callable[[Box[bool]], None]
OnSerialDeviceConnectedDelegate = Callable[[], None]
OnSerialDeviceConnectionErrorDelegate = Callable[[ConnectionErrorReason], None]
OnSerialDeviceDisconnectedDelegate = Callable[[], None]

OnSerialDeviceConnecting = Event[OnSerialDeviceConnectingDelegate]
OnSerialDeviceConnected = Event[OnSerialDeviceConnectedDelegate]
OnSerialDeviceConnectionError = Event[OnSerialDeviceConnectionErrorDelegate]
OnSerialDeviceDisconnected = Event[OnSerialDeviceDisconnectedDelegate]

class SerialDevice:
    def __init__(
        port: str,
        configuration: Optional[SerialDeviceConfiguration] = None
    ): pass

    # Called before connecting. Allows abortion of connection
    on_connecting: OnSerialDeviceConnecting

    # Called when connected to device
    on_connected: OnSerialDeviceConnected

    # Called on every connection error with cause
    on_connection_error: OnSerialDeviceConnectionError

    # Called when device is disconnected
    on_disconnected: OnSerialDeviceDisconnected

    # Current state of connection
    @property
    def connected(self) -> bool: pass
    
    # Connect to serial device
    def connect(self) -> None: pass

    # Disconnects from serial device
    def disconnect(self) -> None: pass

    # Send command to serial device. Returns message of None if error occurred
    def send_command(self, command: str) -> Optional[str]: pass

    # Reads message from serial device. Returns message of None if error occurred
    def read_message(self) -> Optional[str]: pass
```

### SerialDeviceConfiguration

Stores the settings and configuration for serial device communication.

```py
@dataclass
class SerialDeviceConfiguration:
    baudrate: int = 115200
    bytesize: int = 8
    stopbits: int = 1
    timeout: float = 0.1
```

### RadCon

Represents the RadCon devices, handles connecting, reconnecting and reading data from device.

```py

OnRadConConnectedDelegate = Callable[[], None]
OnRadConDisconnectedDelegate = Callable[[], None]
OnRadConDataDelegate = Callable[[datetime, float, int], None]

OnRadConConnected = Event[OnRadConConnectedDelegate]
OnRadConDisconnected = Event[OnRadConDisconnectedDelegate]
OnRadConData = Event[OnRadConDataDelegate]

class RadCon:
    def __init__(
        self,
        port: str,
        reconnect_cooldown: float = 1,
        device: Optional[SerialDevice] = None
    ): pass

    # Called when RadCon device is connected
    on_connected: OnRadConConnected

    # Called when RadCon device is disconnected
    on_disconnected OnRadConDisconnected

    # Called when new data from RadCon device is received
    on_data: OnRadConData
```

## Example usage
This simple scripts tries to get firmware and single measurement from device

```py
def on_data(timestamp, hw_timestamp, pulse_length):
    print("Received data {} {} {}".format(
        timestamp, hw_timestamp, pulse_length
    ))

if __name__ == "__main__":
    r = RadCon("COM3", reconnect_cooldown=1)
    r.on_data.add(on_data)

    # Stars the RadCon
    r.start()

    # Active waiting (RadCon is operated by other threads)
    while True:
        time.sleep(1)
```