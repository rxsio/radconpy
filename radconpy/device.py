from enum import Enum
import logging
import time
from typing import Optional, Union

import serial
import serial.serialutil

from radconpy.errors import RadConNotConnectedException, RadConUnknownException

# region Commands

class RadConCommands(Enum):
    Firmware = "i\r\n"
    
# endregion

# region Device
    
class RadConDevice:

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        bytesize: int = 8,
        stopbits: int = 1,
        timeout: Optional[float] = 0.1,
        logger_level: str = "INFO",
    ):
        """
        Parameters
        ----------
        port : str
            Port on which RadCon device is connected

        baudrate : int = 115200
            Baudrate for serial communication
        bytesize : int = 8
            Bytesize for serial communication
        stopbits : int = 1
            Stopbits for serial communication
        timeout : Optional[float] = 0.1
            Timeout for serial communication. None if no timeout
        logger_level : str = "INFO"
            Logger level
        """
        self._port = port
        self._baudrate = baudrate
        self._bytesize = bytesize
        self._stopbits = stopbits
        self._timeout = timeout
        self._serial = None
        self._is_open = False

        logging.basicConfig(format="%(asctime)s [%(name)s / %(levelname)s] %(message)s")
        self._logger = logging.getLogger("RadCon")
        self._logger.setLevel(
            logging.getLevelNamesMapping().get(logger_level, logging.INFO)
        )
        
    def is_connected(self) -> bool:
        """
        Check if device is connected
        
        Returns
        -------
        bool
            True if device is connected otherwise False
        """
        return self._serial.is_open and self._is_open

    def connect(self) -> bool:
        """
        Connect to RadCon device

        Returns
        -------
        bool
            True if RadCon device is connected successfully otherwise False
        """
        self.disconnect()
        self._logger.info("Attempting to connect to RadCon device")

        # Setup serial connection to device
        try:
            self._serial = serial.Serial(
                self._port,
                baudrate=self._baudrate,
                bytesize=self._bytesize,
                stopbits=self._stopbits,
                timeout=self._timeout,
            )
        except Exception:
            self._logger.warning("Cannot connect to RadCon device")
            self._is_open = False
            return False

        self._logger.info("Connected to RadCon device")
        self._is_open = True
        return True

    def disconnect(self):
        """
        Disconnect RadCon device
        """
        self._is_open = False
        
        # Check if serial exists
        if self._serial is None:
            return
        
        # Check if it is not already closed
        if self._serial.closed:
            return

        # Close serial
        self._serial.close()
        self._logger.info("Disconnected from RadCon device")
        
    def send_command(self, command: Union[str, RadConCommands]) -> str:
        """
        Send command to RadCon device
        
        Parameters
        ----------
        command : str | RadConCommands 
            Command to send
            
        Returns
        -------
        str 
            Response for the command
        """
        if isinstance(command, RadConCommands):
            command = command.value
        
        raw_command = command.encode("ascii")
        self._logger.debug(f"Sending command \"{raw_command}\"")

        # Check if serial is open
        if self._serial is None or not self._serial.is_open:
            self._is_open = False
            raise RadConNotConnectedException()
        
        # Catch all exceptions
        try:
            # Clear the input buffer and send command
            self._serial.read(self._serial.in_waiting)
            self._serial.write(raw_command)
        except (serial.serialutil.PortNotOpenError, serial.serialutil.SerialException):
            self._is_open = False
            raise RadConNotConnectedException()
        except Exception:
            self._is_open = False
            raise RadConUnknownException()

        # Wait for response
        time.sleep(0.1)
        
        # Read line with response
        response = self.readline()
        self._logger.debug(f"Receiving command response: \"{response.encode("ascii")}\"")
        return response
    
    
    def readline(self) -> str:
        """
        Readline from serial device. Line ends with \r\n
        
        Returns
        -------
        str
            Line contents without line ending
        """
        line = ""
        
        # Catch all exceptions
        try:
            # Read until \r\n (end of line)
            while not line.endswith("\r\n"):
                byte = self._serial.read(1)
                line += byte.decode("ascii")
        except serial.serialutil.SerialException:
            self._is_open = False
            raise RadConNotConnectedException()
        except Exception:
            self._is_open = False
            raise RadConUnknownException()
            
        # Return line withount line ending
        return line[:-2]

# endregion