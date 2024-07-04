from enum import Enum
import logging
import time
from typing import Optional, Union

import serial
import serial.serialutil

# region Exceptions 

class RadConNotConnectedException(Exception):
    ...

class RadConUnknownException(Exception):
    ...

# endregion

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
        timeout: Optional[float] = None,
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

# region RadConManager

class RadConManager:
    
    def __init__(self, device: RadConDevice, reconnect_cooldown: float = 1, logger_level: str = "INFO"):
        self._device = device
        self._reconnect_cooldown = reconnect_cooldown
        
        logging.basicConfig(format="%(asctime)s [%(name)s / %(levelname)s] %(message)s")
        self._logger = logging.getLogger("RadConManager")
        self._logger.setLevel(
            logging.getLevelNamesMapping().get(logger_level, logging.INFO)
        )
        
    def ensure_connected(self):
        self._device.connect()
        
    def get_firmware(self, max_tries: int = 3) -> Optional[str]:
        self._logger.debug("Get firmware")
        
        tries = 0
        while tries <= max_tries:
            try:
                firmware = self._device.send_command(RadConCommands.Firmware)
                self._logger.debug(f"Firmware version: {firmware}")
                return firmware
            except (RadConNotConnectedException, RadConUnknownException):
                self.ensure_connected()
            tries += 1
            
            if tries <= max_tries:
                self._logger.debug(f"Cannot get firmware version. Try: {tries} of {max_tries}")
                time.sleep(self._reconnect_cooldown)
            
        self._logger.debug("Firmware version cannot be queryed")
        return None
            

# endregion

if __name__ == "__main__":
    r = RadConDevice("COM3", logger_level="DEBUG")
    m = RadConManager(r, logger_level="DEBUG", reconnect_cooldown=2.0)
    print(m.get_firmware(10))
    # r.connect()
    # rsp = r.send_command(RadConCommands.Firmware)
    
    # while True:
    #     l = r.readline()
    #     print(l)
    
    # r.disconnect()
    # print(rsp)
