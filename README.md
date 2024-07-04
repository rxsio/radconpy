# RadConPy
Library for communicating with RadCon device made in pure Python

## Summary

`RadConDevice`
- a little abstraction over serial communication
- allows to connect / disconnect from device
- allows sending commands and reading lines
- some methods can throw RadConNotConnectedException or RadConUnknownException, when device is not connected to computer

`RadConManager`
- simple manager, that use `RadConDevice` and query useful information
- all methods for quering can return None if 

## Interface

```py
RadConDevice(
    port: str, # Port on which device is connected,
    baudrate: int = 115200, # Baudrate value for serial connection. SHOULD NOT BE CHANGED!
    bytesize: int = 8, # Bytesize for serial connection. SHOULD NOT BE CHANGED!
    stopbits: int = 1, # Stopbits for serial connection. SHOULD NOT BE CHANGED!
    timeout: float = 0.1 # Timeout for reading / writing. Pass None if should block device
    logger_level: str = "INFO" # Level of logging
)
```

`RadConDevice.connect()` - connects to device, returns if it connected successfully
`RadConDevice.disconnect()` - disconnects device
`RadConDevice.send_command(command)` - sends commands to device and received response
`RadConDevice.readline()` - reads line from device

```py
RadConManager(
    device: RadConDevice, # Device which should be managed
    reconnected_cooldown: float = 1, # Reconnect cooldown in seconds
    logger_level: str = "INFO" # Level of logging
)
```

`RadConManager.ensure_connected` - ensures device is connected
`RadConManager.get_firmware(max_tries = 3)` - get firmware of RadCon device
`RadConManager.get_measurement(max_tries = 3)` - get measurement from RadCon device

```py
class Measurement:
    timestamp: datetime.datetime # Computer timestamp of measurement
    hardware_timestamp: float # RadCon timestamp of measurement [milliseconds]
    pulse_length: int # Pulse length measured by RadCon device [microseconds]
```