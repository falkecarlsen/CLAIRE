#!/usr/bin/env python
"""
Script for reproducing the experiment done on the BUILD setup.

Replication of experiment 15, which has the following parameter settings.
- Experiment duration: 140 min.
- Rainfall data: 20 minutes of 0.1 my-m/s rain, followed by the first 120 minutes of rain data.
- Initial water level: 650 mm.
- Physical water limit (overflow level): 800 mm.
- Duration control period: 10 min.
- Control change interval: 1 min.
- Control horizon: 70 min.
- Optimization cost function: E(alpha*o + s + c), with alpha = 10 000.
- Fixed outflow setting: 2.
- Learning budget parameters: --good-runs 200 --total-runs 200 --runs-pr-state 100 --eval-runs 100.
- Discretization: 0.03.

For our demonstrator setup, the overflow level is approx 600 mm.
So initial level should be 450 mm.
TODO We could consider scaling the dimensions, i.e., 1mm in build setup is 2 mm in demonstrator setup. So in- and
outflows can be a bit bigger by this.
"""
import threading

import driver
from time import time, sleep
from enum import Enum


# Insert the name of the usb port, which might be different for different devices.
# An easy way to get the port name is to use the Arduino IDE.
# PORT = '/dev/ttyUSB0'
PORT = '/dev/cu.usbmodem14201'

# File to write calibration data to
data_file = "build_data.txt"
weather_data_file = "Rainfall.txt"

# Global object of the claire setup.
claire = driver.ClaireDevice(PORT)

height_scale = 2
time_scale = 1
max_water_level = 600  # TODO calibrate this number.
duration = 10 * 60 / time_scale  # Duration in seconds


class OutflowSettings(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3


class StopwatchStates(Enum):
    READY = 1
    RUNNING = 2
    FINISHED = 3


class Stopwatch:
    def __init__(self, start_state=StopwatchStates.READY):
        self.sleep_period = 0.05
        self.state = start_state
        self.end_time = time()
        self.stopped = False

        self.timer_thread = threading.Thread(target=self._wait_)
        self.timer_thread.daemon = True
        self.timer_thread.start()

    def _wait_(self):
        while True:
            if self.state == StopwatchStates.RUNNING:
                if time() < self.end_time:
                    sleep(self.sleep_period)
                else:
                    self.state = StopwatchStates.FINISHED

            if self.stopped:
                break

    def start(self, wait_duration):
        if self.state != StopwatchStates.READY:
            print("Stopwatch was not ready, reset and started a new one.")
        self.end_time = time() + wait_duration
        self.state = StopwatchStates.RUNNING

    def is_finished(self):
        if self.state == StopwatchStates.FINISHED:
            self.state = StopwatchStates.READY
            return True
        return False

    def stop(self):
        self.stopped = True


def run_experiment():
    """The main experiment function. Closes the demonstrator connection when finished."""
    print("Resetting tubes to initial value.")
    # reset_tubes()
    rainfall = [1.0] * 20  # First 20 minutes light rainfall.
    rainfall.extend(read_weather_data())  # Then follow rain data.

    # Start experiment.
    experiment_loop(rainfall)

    claire.close()


def experiment_loop(rainfall):
    print("Start experiment.")
    start_time = time()
    finish_time = start_time + duration

    rainfall_reading_stopwatch = Stopwatch(start_state=StopwatchStates.FINISHED)
    rain_fraction = get_rain_on(rainfall[0], 1)  # Fraction of time that it should be raining.
    rainfall_stopwatch = Stopwatch(start_state=StopwatchStates.FINISHED)
    inflow_on = False
    outflow_stopwatch = Stopwatch(start_state=StopwatchStates.FINISHED)
    outflow_on = False

    while time() < finish_time:
        # Run the experiment.

        # Start with reading rainfall data
        if rainfall_reading_stopwatch.is_finished():
            rain_index = int((time() - start_time) * time_scale // 60)  # // is floor division
            rain_fraction = get_rain_on(rainfall[rain_index], 1)
            rainfall_reading_stopwatch.start(60 / time_scale)  # Rainfall data is every minute.

        # Adjust rainfall.
        if rainfall_stopwatch.is_finished():
            if inflow_on:
                claire.set_inflow(1, 0)
                inflow_on = False
                rainfall_stopwatch.start(59.5 / time_scale * (1 - rain_fraction))
            else:
                claire.set_inflow(1, 100)
                inflow_on = True
                print(59.5 / time_scale * rain_fraction + 0.5)
                rainfall_stopwatch.start(59.5 / time_scale * rain_fraction + 0.5)

        # Adjust outflow.
        if outflow_stopwatch.is_finished():
            if outflow_on:
                claire.set_outflow(1, 0)
                outflow_on = False
                outflow_stopwatch.start(60 / time_scale * (1 - get_outflow_on(OutflowSettings.MEDIUM, 1)))
            else:
                claire.set_outflow(1, 100)
                outflow_on = True
                outflow_stopwatch.start(60 / time_scale * get_outflow_on(OutflowSettings.MEDIUM, 1))

        sleep(0.1)

    print("Finished with experiment.")
    claire.set_outflow(1, 0)
    claire.set_outflow(2, 0)
    claire.set_inflow(1 ,0)
    claire.set_inflow(2, 0)
    rainfall_reading_stopwatch.stop()
    rainfall_stopwatch.stop()
    outflow_stopwatch.stop()


def reset_tubes():
    claire.set_water_level(1, max_water_level - height_scale * 150)
    claire.wait_until_free()
    claire.set_water_level(2, max_water_level - height_scale * 150)
    claire.wait_until_free()


def read_weather_data() -> list[float]:
    rainfall = []
    with open(weather_data_file, 'r') as f:
        for line in f:
            rainfall.append(float(line))
    return rainfall


def get_rain_on(intensity, tube):
    if tube == 1:
        return intensity * height_scale * time_scale * 9.282 / 1000
    elif tube == 2:
        return intensity * height_scale * time_scale * 8.544 / 1000
    else:
        raise RuntimeError("Undefined tube.")


def get_outflow_on(setting, tube):
    assert tube == 1 or tube == 2
    assert isinstance(setting, OutflowSettings)

    if setting == OutflowSettings.LOW:
        if tube == 1:
            percentage_on = 0.112
        elif tube == 2:
            percentage_on = 0.118
    elif setting == OutflowSettings.MEDIUM:
        if tube == 1:
            percentage_on = 0.187
        elif tube == 2:
            percentage_on = 0.196
    elif setting == OutflowSettings.HIGH:
        if tube == 1:
            percentage_on = 0.431
        elif tube == 2:
            percentage_on = 0.451

    return percentage_on * height_scale * time_scale / 100  # /100 to get from percentage to fraction.


if __name__ == '__main__':
    try:
        run_experiment()  # run_experiment will already close the connection when properly finished.
    except driver.SensorError:
        claire.write("3;")  # Emergency stop
        claire.close()
        raise RuntimeError("Sensor reading failed. Please run experiment again.")
    except KeyboardInterrupt:
        claire.wait_until_free()
        claire.write("3;")  # Emergency stop
        claire.close()

