import logging
from dataclasses import dataclass
from typing import Optional

import serial

from radconpy.api import Box
from radconpy.hardware.events import OnSerialDeviceConnecting, \
    OnSerialDeviceConnected, OnSerialDeviceDisconnected, \
    OnSerialDeviceConnectionError, ConnectionErrorReason

logger = logging.getLogger("radconpy")
logger.setLevel(logging.DEBUG)


@dataclass
class SerialDeviceConfiguration:
    """
    Represents a serial device configuration.

    Attributes
    ----------
    baudrate : int = 115200
        The speed at which data is transmitted over the serial interface.
    bytesize : int = 8
        The number of data bits in each character.
    stopbits : int = 1
        The number of stop bits used to indicate the end of a character.
    timeout : float = 0.1
        The time (in seconds) to wait for a response before timing out.
        If no response is received within this time, the communication
        is considered to have failed.
    """
    baudrate: int = 115200
    bytesize: int = 8
    stopbits: int = 1
    timeout: float = 0.1


class SerialDevice:

    def __init__(
            self,
            port: str,
            configuration: Optional[SerialDeviceConfiguration] = None
    ):
        """
        Initialise a serial device.

        Parameters
        ----------
        port : str
            Port name of the serial device to connect to.
        configuration : Optional[SerialDeviceConfiguration]
            Serial configuration
        """
        self._port = port
        self._configuration = configuration \
            if configuration is not None \
            else SerialDeviceConfiguration()

        self._serial = None

        # Events
        self.on_connecting = OnSerialDeviceConnecting()
        self.on_connected = OnSerialDeviceConnected()
        self.on_connection_error = OnSerialDeviceConnectionError()
        self.on_disconnected = OnSerialDeviceDisconnected()

        # Event Listeners
        def on_error(reason: ConnectionErrorReason):
            self.disconnect()

        def on_disconnect():
            self._serial = None

        self.on_connection_error.add(on_error)
        self.on_disconnected.add(on_disconnect)

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        """
        Connects to a serial device
        """
        if self.connected:
            return

        should_allow = Box(True)
        logger.debug("Requesting connecting to serial device")
        self.on_connecting.broadcast(should_allow)

        if not should_allow.get():
            logger.debug("Connecting to serial device aborted by user")
            self.on_connection_error.broadcast(
                ConnectionErrorReason.ConnectionAborted
            )
            return

        try:
            self._serial = serial.Serial(
                self._port,
                baudrate=self._configuration.baudrate,
                bytesize=self._configuration.bytesize,
                stopbits=self._configuration.stopbits,
                timeout=self._configuration.timeout
            )
        except Exception as e:
            logger.error(f"Cannot connect to serial device. Cause: {e}")
            self.on_connection_error.broadcast(
                ConnectionErrorReason.ConnectionAttemptFailed
            )
            return

        logger.info("Connected to serial device")
        self.on_connected.broadcast()

    def disconnect(self) -> None:
        """
        Disconnects the serial device
        """
        try:
            if self._serial is not None and not self._serial.closed:
                self._serial.close()
        finally:
            logger.info("Disconnected from serial device")
            self.on_disconnected.broadcast()

    def send_command(self, command: str) -> Optional[str]:
        """
        Sends a command to the RadCon device

        Parameters
        ----------
        command : str
            Command to send to RadCon device

        Returns
        -------
        Optional[str]
            Command response or None if exception occurred
        """

        logger.debug(f"Sending command \"{command}\" to serial device")

        if not self.connected:
            return None

        try:
            self._serial.read(self._serial.in_waiting)
            self._serial.write(command.encode("ascii"))
        except (serial.serialutil.PortNotOpenError,
                serial.serialutil.SerialException) as e:
            logger.error(f"Serial exception during sending command: {e}")
            self.on_connection_error.broadcast(
                ConnectionErrorReason.ConnectionLost
            )
            return None
        except Exception as e:
            logger.error(f"Exception during sending command to serial: {e}")
            self.on_connection_error.broadcast(
                ConnectionErrorReason.Other
            )
            return

        response = self.read_message()
        logger.debug(f"Received response for command: {response}")
        return response

    def read_message(self, token: Box[bool] = None) -> Optional[str]:
        """
        Reads a message from the serial device

        Returns
        -------
        Optional[str]
            Message from device or None if exception occurred
        """
        if not self.connected:
            return None
        
        if token is None:
            token = Box(True)

        line = ""

        logger.debug("Reading message from serial device")

        try:
            while not line.endswith("\r\n") and self.connected and token.value:
                byte = self._serial.read(1)

                if byte != b'':
                    line += byte.decode("ascii")
        except serial.serialutil.SerialException as e:
            logger.error(f"Serial exception during reading: {e}")
            self.on_connection_error.broadcast(
                ConnectionErrorReason.ConnectionLost
            )
            return None
        except Exception as e:
            logger.error(f"Exception during reading from serial: {e}")
            self.on_connection_error.broadcast(
                ConnectionErrorReason.Other
            )
            return None

        return line[:-2]
