# temperature_simulator.py

import math
import random
import time


class TemperatureSimulator:
    def __init__(self, min_temp=35.0, max_temp=38.0):
        self.min_temp = min_temp
        self.max_temp = max_temp
        self.start_time = time.time()

    def get_temperature(self):
        elapsed = time.time() - self.start_time
        amplitude = (self.max_temp - self.min_temp) / 2
        midpoint = (self.max_temp + self.min_temp) / 2

        # Sine wave + random noise
        value = midpoint + amplitude * math.sin(2 * math.pi * elapsed / 60.0)
        noise = random.uniform(-0.2, 0.2)
        return value + noise
