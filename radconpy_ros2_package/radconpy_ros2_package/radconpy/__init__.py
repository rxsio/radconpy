import random
import time

from radcompy_ros2_package.radconpy.collector import RadConCollector
from radcompy_ros2_package.radconpy.device import RadConDevice
from radcompy_ros2_package.radconpy.manager import Measurement, RadConManager


class MockRadConManager(RadConManager):

    def get_measurement(self, max_tries: int = 3) -> Measurement | None:
        time.sleep(random.randint(100, 300) / 100)
        return 3
    
    def get_firmware(self, max_tries: int = 3) -> str | None:
        return "MOCK"


if __name__ == "__main__":
    r = RadConDevice("COM3", logger_level="DEBUG")
    m = MockRadConManager(r, logger_level="DEBUG", reconnect_cooldown=2.0)
    c = RadConCollector(m, "m1.csv", timebase=60)

    print(f"Firmware: {m.get_firmware(3)}")
    c.run(visualize=True)
