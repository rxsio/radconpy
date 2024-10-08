import threading
import time
import queue
from datetime import datetime

from radconpy.api import Box
from radconpy.events import OnRadConConnected, OnRadConDisconnected, \
    OnRadConData
from radconpy.hardware.device import SerialDevice


class RadCon:

    def __init__(
            self,
            port: str,
            reconnect_cooldown: float = 1,
            device: SerialDevice = None
    ):
        if device is None:
            device = SerialDevice(port)

        self._running = False
        self._device = device
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
        self._running = True
        self._device.connect()

    def stop(self):
        self._running = False
        self._device.disconnect()
        print('disconnect')
        # Stop thread
