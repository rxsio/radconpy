from dataclasses import dataclass
from datetime import datetime
from threading import Thread
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.animation import FuncAnimation

from radconpy.manager import RadConManager


@dataclass
class RadConStatistics:
    timestamp: datetime
    cpm: float
    doserate: float


class RadConCollector:
    """
    Collects statistics for RadCon device
    """

    def __init__(self, manager: RadConManager, calibration_factor: float = 0.005, timebase: float = 60):
        """
        Parameters
        ----------
        manager : RadConManager
            Manager of the RadCon device
        calibration_factor : float = 0.005
            Calibration factor for calculating doserate [Gy / CPM]
        timebase : float = 60
            Basic timebase (time resolution) of statistics [seconds]
        """
        self._manager = manager
        self._calibration_factor = calibration_factor
        self._timebase = timebase

        self._count = 0
        self._lock = threading.Lock()
        self._statistics: list[RadConStatistics] = []

        self._is_alive = True
        self._collecting_thread = Thread(target = self.collect, daemon=True)
        self._calculation_thread = Thread(target = self.calculate, daemon=True)
        self._visualization_thread = Thread(target = self.visualize, daemon=True)

    def run(self, visualize: bool = False):
        """
        Run collecting statistics
        """
        self._collecting_thread.start()
        self._calculation_thread.start()

        if visualize:
            self._visualization_thread.start()

        try:
            while True:
                time.sleep(0)
        except KeyboardInterrupt:
            self._is_alive = False

        self._collecting_thread.join()
        self._calculation_thread.join()
        
        if visualize:
            self._visualization_thread.join()

    def collect(self):
        """
        Data collection thread
        """
        while self._is_alive:
            measurement = self._manager.get_measurement()

            if measurement is not None:
                with self._lock:
                    self._count += 1
                print(self._count)
        
    def calculate(self):
        """
        Calculation thread
        """
        processing_time = 0

        while self._is_alive:
            time.sleep(self._timebase - processing_time)
            start = time.perf_counter()

            cps = self._count / self._timebase
            cpm = cps * 60
            doserate = cpm * 60 * 24 * self._calibration_factor

            with self._lock:
                self._statistics.append(
                    RadConStatistics(
                        timestamp = datetime.now(),
                        cpm=cpm,
                        doserate=doserate
                    )
                )

                self._count = 0
            print("cpm=", cpm)

            processing_time = time.perf_counter() - start

    def visualize(self):
        """
        Visualization thread
        """
        def animate(i):
            with self._lock:
                if len(self._statistics) > 0:
                    times = [stat.timestamp for stat in self._statistics]
                    cpms = [stat.cpm for stat in self._statistics]
                    dose_rates = [stat.doserate for stat in self._statistics]

                    ax1.clear()
                    ax2.clear()

                    ax1.plot(times, cpms, marker='o', linestyle='-', color='b')
                    ax2.plot(times, dose_rates, marker='o', linestyle='-', color='r')

                    ax1.set_title('CPM over Time')
                    ax1.set_ylabel('CPM')
                    ax2.set_title('Dose Rate over Time')
                    ax2.set_ylabel('Dose Rate (Gy/d)')

                    ax1.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
                    ax2.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))

                    fig.autofmt_xdate()

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        ani = FuncAnimation(fig, animate, interval=1000)
        plt.tight_layout()
        plt.show()