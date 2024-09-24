import threading
import time
import queue
from datetime import datetime
from collections import deque

from radconpy.api import Box
from radconpy.events import OnRadConConnected, OnRadConDisconnected, \
    OnRadConData
from radconpy.hardware.device import SerialDevice


class RadCon:

    def __init__(
            self,
            port: str,
            reconnect_cooldown: float = 1,
            cpm_window_width: float = 60,
            device: SerialDevice = None
    ):
        """
        Initialize a RadCon device

        Parameters
        ----------
        port : str
            Port on which RadCon is connected
        reconnect_cooldown : float = 1.0
            Cooldown between reconnections
        cpm_width_width : float = 60
            Width of the CPM (Counts per Minute) sliding window in seconds
        device : Optional[SerialDevice] = None
            Serial device on which RadCon is connected
        """
        if device is None:
            device = SerialDevice(port)

        self._running = False
        self._device = device
        self._pulse_next_update = datetime.now()
        self._pulse_timestamps = deque()
        self._cpm_window_width = cpm_window_width
        self._reconnect_cooldown = reconnect_cooldown

        self._lock = threading.Lock()
        self._token = Box(True)
        self._command_queue = queue.Queue()
        self._command_queue_response = queue.Queue()

        self.on_connected = OnRadConConnected()
        self.on_disconnected = OnRadConDisconnected()
        self.on_data = OnRadConData()

        self._thread = None
        self._reconnect_thread = None

        # Event Listeners
        self._device.on_connected.add(self._on_connected)
        self._device.on_disconnected.add(self._on_disconnected)
        self.on_data.add(self._on_data)

    @property
    def cpm(self) -> float:
        """
        Measured CPM (Counts per Minute)

        Returns
        -------
        float
            Counts per Minute
        """
        if len(self._pulse_timestamps) == 0:
            return 0
        
        self._filter_pulses()

        return len(self._pulse_timestamps) / self._cpm_window_width * 60

    @property
    def cpm_window_width(self) -> float:
        """
        Width of the sliding window of CPM (Counts per Minute)

        Returns
        -------
        float
            Window width in seconds
        """
        return self._cpm_window_width

    @cpm_window_width.setter
    def cpm_window_width(self, value: float) -> None:
        """
        Sets width of the CPM (Counts per Minute) sliding window
        
        Parameters
        ----------
        seconds : float
            Width of the sliding window
        """
        if value <= 0:
            raise ValueError(
                "Sliding window width cannot be less or equal than zero"
            )
        
        self._pulse_next_update = datetime.now()
        self._cpm_window_width = value

    def send_command(self, command: str) -> None:
        with self._lock:
            self._command_queue.put(command)
            self._token.set(False)
            value = self._command_queue_response.get()

        return value

    def _on_connected(self):
        if not self._running:
            return

        self.on_connected.broadcast()

        self._thread = threading.Thread(target=self._worker)
        self._thread.start()

    def _on_disconnected(self):
        if not self._running:
            return

        self.on_disconnected.broadcast()

        if self._thread is not None:
            self._thread.join()

        time.sleep(self._reconnect_cooldown)
        self._reconnect_thread = threading.Thread(target=self._reconnect)
        self._reconnect_thread.start()

    def _on_data(self, timestamp, hardware_timestamp, pulse_length):
        if len(self._pulse_timestamps) == 0:
            self._pulse_next_update = timestamp + datetime.timedelta(seconds=self._cpm_window_width)

        if len(self._pulse_next_update) > 1000:
            self._filter_pulses()

        self._pulse_timestamps.append(timestamp)

    def _self_filter_pulses(self) -> None:
        current = datetime.now()
        if current <= self._pulse_next_update:
            return
        
        while len(self._pulse_timestamps) > 0 and self._pulse_timestamps[0] < current:
            self._pulse_timestamps.popleft()

        if len(self._pulse_timestamps) > 0:
            self._pulse_next_update = self._pulse_timestamps[0]

    def _reconnect(self):
        self._device.connect()

    def _worker(self):
        while self._device.connected:
            message = self._device.read_message(self._token)

            if not self._token.value:
                while not self._command_queue.empty():
                    command = self._command_queue.get()
                    response = self._device.send_command(command)
                    self._command_queue_response.put(response)

                self._token.set(True)
                continue

            if message is None:
                break

            values = message.split(" ")
            if len(values) != 2:
                # Log invalid
                continue

            try:
                hardware_timestamp = float(values[0])
                pulse_length = int(values[1])
            except ValueError:
                # Log ivnalid data
                continue

            self.on_data.broadcast(
                datetime.now(),
                hardware_timestamp,
                pulse_length
            )

    def start(self):
        """
        Connects to RadCon and starts observing
        """
        self._running = True
        self._device.connect()

    def stop(self):
        """
        Disconnects from RadCon and stops observign
        """
        self._running = False
        self._device.disconnect()
