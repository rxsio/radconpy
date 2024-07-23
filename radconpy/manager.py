from dataclasses import dataclass
import datetime
import logging
import time
from typing import Optional

from radconpy.device import RadConCommands, RadConDevice
from radconpy.errors import RadConNotConnectedException, RadConUnknownException


@dataclass
class Measurement:
    """
    Class for storing measurement
    
    Fields
    ------
    timestamp : datetime.datetime
        Computer timestamp of measurement
    hardware_timestamp : float
        Timestamp of RadCon device in milliseconds
    pulse_length : int
        Length of pulse in microseconds
    """
    timestamp: datetime.datetime
    hardware_timestamp: float
    pulse_length: int


class RadConManager:
    """
    Manager for managing RadConDevice
    """
    
    def __init__(self, device: RadConDevice, reconnect_cooldown: float = 1, logger_level: str = "INFO"):
        """
        Parameters
        ----------
        device : RadConDevice
            Device to manage
        reconnect_colldown : float = 2
            Reconnect cooldown
        logger_level : str = "INFO"
            Logger level
        """
        self._device = device
        self._reconnect_cooldown = reconnect_cooldown
        
        logging.basicConfig(format="%(asctime)s [%(name)s / %(levelname)s] %(message)s")
        self._logger = logging.getLogger("RadConManager")
        self._logger.setLevel(
            logging.getLevelNamesMapping().get(logger_level, logging.INFO)
        )
        
        self.ensure_connected()
        
    def ensure_connected(self):
        """
        Ensure the device is connected
        """
        self._device.connect()
        
    def get_firmware(self, max_tries: int = 3) -> Optional[str]:
        """
        Get firmware version of RadCon device
        
        Parameters
        ----------
        max_tries : int = 3
            Max additional tries of querying information
        
        Returns
        -------
        Optional[str]
            Firmware version or None if cannot query firmware version
        """
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
            
        self._logger.warning("Firmware version cannot be queryed")
        return None
    
    def get_measurement(self, max_tries: int = 3) -> Optional[Measurement]:
        """
        Get measurement in user-friendly way
        
        Parameters
        ----------
        max_tries : int = 3
            Max additional tries for reconnections
            
        Returns
        -------
        Optional[Measurement]
            Measurement of None if it cannot be quered
        """
        self._logger.debug("Get measurement")
        
        tries = 0
        line = None
        while tries <= max_tries:
            try:
                line = self._device.readline()
                break
            except (RadConNotConnectedException, RadConUnknownException):
                self.ensure_connected()
                
            tries += 1
            
            if tries <= max_tries:
                self._logger.debug(f"Cannot get measurement. Try: {tries} of {max_tries}")
                time.sleep(self._reconnect_cooldown)
    
        if line is None:
            self._logger.warning("Measurement cannot be get")
            return None
        
        values = line.split(" ")
        if len(values) != 2:
            self._logger.debug(f"Invalid data received (too few values): {line}")
            return None
        
        try:
            timestamp = float(values[0])
            pulse_length = float(values[1])
        except ValueError:
            self._logger.debug(f"Invalid data received (cannot convert to numbers): {line}")
            return None
        
        return Measurement(datetime.datetime.now(), timestamp, pulse_length)