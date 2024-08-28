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
data_file = "calibration_outflow_data.txt"

# Global object of the claire setup.
claire = driver.ClaireDevice(PORT)


def calibration():
    """The main calibration function. Closes the demonstrator connection when finished."""
    reset_tubes()

    with open(data_file, 'w') as f:
        f.write("Tube,isInflow,duration[s],difference[mm],rate[mm/s],duty\n")
        # Switch between tube 1 and tube 2 to let the solenoids cool down a bit.
        start_time = time()
        for duty_level in range(200):
            claire.state.make_outdated()
            state = claire.get_state()
            f.write(f"{time() - start_time},{state['Tube0_water_mm']},{state['Tube1_water_mm']}\n")

    claire.close()


def reset_tubes():
    claire.set_water_level(1, 500)
    claire.wait_until_free()
    claire.set_water_level(2, 500)
    claire.wait_until_free()


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

