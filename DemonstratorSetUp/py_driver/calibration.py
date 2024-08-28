#!/usr/bin/env python
"""
Script for automatic calibration measurements of the pumps in the demonstrator.

This script calibrates the inflow and outflow pump of each tube for different duty levels. Each combination of
tube-flow-duty is repeated 5 times. It alternates between the tubes to let the outflow solenoid cool down. The limited
range of water levels of approximately 250 mm to 500 mm is used. A lower water level can cause sonar measurement
interference.

Total calibration experiment takes about 2.5 hours. Supervision is needed, as potential communication errors between
demonstrator setup and PC terminate this python script. To continue after communication error, inspect the `data_file`
for last run experiment, and adjust the start `duty_level` in :func:`calibration()`. Make sure that you rename the
already obtained data file, as this script overwrites old data without warning.
"""

import driver
from time import time


# Insert the name of the usb port, which might be different for different devices.
# An easy way to get the port name is to use the Arduino IDE.
# PORT = '/dev/ttyUSB0'
PORT = '/dev/cu.usbmodem14201'

# File to write calibration data to
data_file = "calibration_data.txt"

# Global object of the claire setup.
claire = driver.ClaireDevice(PORT)


def calibration():
    """The main calibration function. Closes the demonstrator connection when finished."""
    with open(data_file, 'w') as f:
        f.write("Tube,isInflow,duration[s],difference[mm],rate[mm/s],duty\n")
        # Switch between tube 1 and tube 2 to let the solenoids cool down a bit.
        for duty_level in range(10, 101, 10):
            calibration_tube(1, duty_level, f)
            calibration_tube(2, duty_level, f)
    claire.close()


def calibration_tube(tube, duty_level, file):
    """
    Calibration of a tube for a particular duty level.

    :param tube: The tube ID number. Must be value in range [1,2].
    :param duty_level: The duty level to calibrate for. Must be value in range [0,100].
    :param file: The file to write the results to.
    """
    claire.state.make_outdated()  # Start each calibration of a tube with 'outdated' data.
    for i in range(5):
        calibration_inflow(tube, duty_level, file)
        calibration_outflow(tube, duty_level, file)


def calibration_inflow(tube, duty_level, file):
    """
    Calibration of the inflow of a tube.

    :param tube: The tube ID number. Must be value in range [1,2].
    :param duty_level: The duty level to calibrate for. Must be value in range [0,100].
    :param file: The file to write the results to.
    """
    calibration_flow(tube, 250, 500, duty_level, file)


def calibration_outflow(tube, duty_level, file):
    """
    Calibration of the outflow of a tube.

    :param tube: The tube ID number. Must be value in range [1,2].
    :param duty_level: The duty level to calibrate for. Must be value in range [0,100].
    :param file: The file to write the results to.
    """
    calibration_flow(tube, 500, 250, duty_level, file)


def calibration_flow(tube, start_level, end_level, duty_level, file):
    """
    Calibration of a tube. If :param:`end_level` is higher (lower) than :param:`start_level`, inflow (outflow) is being
    calibrated. Any level below (above) :param:`start_level` is a valid start state, while any level above (below)
    :param:`end_level` is a valid end state. If the current state at the beginning of the calibration is not a valid
    start level, the level is first automatically adjusted using the demonstrator's driver function
    :func:`set_water_level`.

    :param tube: The tube ID number. Must be value in range [1,2].
    :param start_level: The start level of the calibration.
    :param end_level: The end level of the calibration.
    :param duty_level: The duty level to calibrate for. Must be value in range [0,100].
    :param file: The file to write the results to.
    """
    print(f"Start flow calibration of tube {tube} with duty level {duty_level}.")
    measure_inflow = start_level < end_level
    state = claire.get_state()

    # Only reset level if it is too high (for inflow) or too low (for outflow)
    if measure_inflow and state[f"Tube{tube - 1}_water_mm"] >= start_level:
        claire.set_water_level(tube, start_level)
    if not measure_inflow and state[f"Tube{tube - 1}_water_mm"] <= start_level:
        claire.set_water_level(tube, start_level)
    claire.wait_until_free()

    print("Start calibration.")
    state = claire.get_state()
    start_time = time()
    true_start_level = state[f"Tube{tube - 1}_water_mm"]
    print(f'Time: {start_time}; level: {true_start_level}')

    # Start in- or outflow
    if measure_inflow:
        claire.set_inflow(tube, duty_level)
    else:
        claire.set_outflow(tube, duty_level)

    # Wait until desired level is reached
    while True:
        try:
            state = claire.get_state()  # Requesting current state takes time, so no need to delay explicitly.
        except driver.SensorError:
            print("Sensor error, trying again.")
            continue

        # Check whether calibration can be finished.
        if measure_inflow and state[f"Tube{tube - 1}_water_mm"] > end_level:
            break
        elif not measure_inflow and state[f"Tube{tube - 1}_water_mm"] < end_level:
            break

    # Stop in- or outflow pump.
    if measure_inflow:
        claire.set_inflow(tube, 0)
    else:
        claire.set_outflow(tube, 0)

    # Final measurement
    end_time = time()
    state = claire.get_state()
    true_end_level = state[f"Tube{tube - 1}_water_mm"]
    print(f'Time: {end_time}; level: {true_end_level}')
    duration = end_time - start_time
    difference = true_end_level - true_start_level
    print(f'Duration: {duration}; change: {difference}')

    # Tube,isInflow,duration[s],difference[mm],rate[mm/s],duty
    rate = difference / duration
    file.write(f"{tube},{measure_inflow},{duration},{difference},{rate},{duty_level}\n")


if __name__ == '__main__':
    try:
        calibration()  # Calibration will already close the connection when properly finished.
    except driver.SensorError:
        claire.write("3;")  # Emergency stop
        claire.close()
        raise RuntimeError("Sensor reading failed. Please run experiment again.")
    except KeyboardInterrupt:
        claire.wait_until_free()
        claire.write("3;")  # Emergency stop
        claire.close()

