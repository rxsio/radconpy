from enum import Enum
import logging
import time
from typing import Optional, Union

import serial
import serial.serialutil

# region Exceptions 

class RadConNotConnectedException(Exception):
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

        logging.basicConfig(format="%(asctime)s [%(levelname)s] %(message)s")
        self._logger = logging.getLogger("RadCon")
        self._logger.setLevel(
            logging.getLevelNamesMapping().get(logger_level, logging.INFO)
        )

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
        except serial.serialutil.SerialException as err:
            self._logger.warn("Cannot connect to RadCon device")
            return False

        self._logger.info("Connected to RadCon device")
        return True

    def disconnect(self):
        """
        Disconnect RadCon device
        """
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
        if isinstance(command, RadConCommands):
            command = command.value
        
        raw_command = command.encode("ascii")
        self._logger.debug(f"Sending command \"{raw_command}\"")

        # Check if serial is open
        if not self._serial.is_open:
            raise RadConNotConnectedException()
        
        # Clear the input buffer and send command
        try:
            self._serial.read(self._serial.in_waiting)
            self._serial.write(raw_command)
        except serial.serialutil.PortNotOpenError:
            raise RadConNotConnectedException()
        except serial.serialutil.SerialException:
            raise RadConNotConnectedException()
        
        # Wait for response
        time.sleep(0.1)
        
        # Read line with response
        response = self.readline()
        self._logger.debug(f"Receiving command response: \"{response.encode("ascii")}\"")
        return response
    
    
    def readline(self) -> str:
        line = ""
        
        # Read until \r\n (end of line)
        while not line.endswith("\r\n"):
            byte = self._serial.read(1)
            line += byte.decode("ascii")
            
        # Return line withount line ending
        return line[:-2]

# endregion

if __name__ == "__main__":
    r = RadConDevice("COM3", logger_level="DEBUG")
    r.connect()
    rsp = r.send_command(RadConCommands.Firmware)
    
    while True:
        l = r.readline()
        print(l)
    
    r.disconnect()
    print(rsp)
