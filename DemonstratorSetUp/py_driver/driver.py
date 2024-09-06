import sys
import threading
from dataclasses import dataclass
from datetime import timedelta, datetime
from time import sleep, time
from typing import Optional

import serial
import utils

IMMEDIATE_OUTPUT = True
TAG = "DRIVER:"
CLAIRE_VERSION = "v0.1.16"
CLAIRE_READY_SIGNAL = "CLAIRE-READY"
TUBE_MAX_LEVEL = 900
DEBUG = True
COMMUNICATION_TIMEOUT = 10
UNDERFLOW_CHECK_INTERVAL = 5


class SensorError(Exception):
    """
    Error when sensor reading fails (i.e., returns -1).
    """
    pass


class ColorPrinting(object):
    """
    ANSI escape coding for printing colors.

    Usage: start string with the desired format escape code, end string with ENDC end coding.
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    @staticmethod
    def print_blue(text):
        """
        Print the text in blue to standard output.

        :param text: The text to print.
        """
        print(f"{ColorPrinting.OKBLUE}{text}{ColorPrinting.ENDC}")


@dataclass
class ClaireState:
    """
    The state of the Claire demonstrator. Can be used to cache the state.
    """
    Tube1_sonar_dist_mm: Optional[float] = None
    Tube2_sonar_dist_mm: Optional[float] = None
    Tube1_inflow_duty: Optional[int] = None
    Tube1_outflow_duty: Optional[int] = None
    Tube2_inflow_duty: Optional[int] = None
    Tube2_outflow_duty: Optional[int] = None
    Stream_inflow_duty: Optional[int] = None
    Stream_outflow_duty: Optional[int] = None
    dynamic: Optional[bool] = None
    last_update: datetime = datetime.now() - timedelta(hours=1)

    def __init__(self, state):
        self.dynamic = False  # set false initially to allow first update to succeed
        self.set_state(state)

    def set_state(self, state):
        """
        Set the cached state to the provided state. Assumes that the provided state is actual.

        :param state: The new state to cache.
        """
        try:
            self.state = state
            for key, value in state.items():
                if hasattr(self, key):
                    setattr(self, key, value)
            self.last_update = datetime.now()
            self.dynamic = state["Tube1_inflow_duty"] or state["Tube1_outflow_duty"] or state["Tube2_inflow_duty"] or \
                           state["Tube2_outflow_duty"]
        except Exception as e:
            print(f"Exception occurred during state update: {e}")
            raise  # Re-raise the exception after handling

    def make_dynamic(self):
        """
        Label the cached state as dynamic.
        When the demonstrator is being acted upon, all state updates are outdated from when they are measured.
        """
        self.dynamic = True

    @staticmethod
    def convert_distance_to_level(distance):
        """
        Convert sensor distance to water level.

        :param distance: The distance from the sensor to the measured water surface.
        """
        if distance < 0:
            raise SensorError()
        return TUBE_MAX_LEVEL - distance

    @staticmethod
    def convert_level_to_distance(level):
        """
        Convert water level to sensor distance.

        :param level: The water level to convert.
        """
        return TUBE_MAX_LEVEL - level

    @property
    def tube1_level(self) -> Optional[float]:
        return self.convert_distance_to_level(self.Tube1_sonar_dist_mm)

    @property
    def tube2_level(self) -> Optional[float]:
        return self.convert_distance_to_level(self.Tube2_sonar_dist_mm)

    def get_tube_level(self, tube):
        if tube == 1:
            return self.tube1_level
        if tube == 2:
            return self.tube2_level
        else:
            raise RuntimeError(f"Unexpected tube number {tube}.")


class ClaireDevice:
    """
    Class that represents the Claire demonstrator setup.
    """
    state: ClaireState

    def __init__(self, port):
        self.e_stop = False
        self.device = port
        self.busy = True  # initially unknown, therefore busy

        # exclusive only available on posix-like systems, assumes mac-env is posix-like
        if ["linux", "darwin"].__contains__(sys.platform):
            exclusive = True
        else:
            exclusive = False

        self.ser = serial.Serial(port, 115200, timeout=1, exclusive=exclusive)

        # buffer of entire run
        self.stopped = False
        self.read_buffer = []
        self.last_printed_buf_line = -1

        self.read_thread = threading.Thread(target=self._read_lines)
        self.read_thread.daemon = True
        self.read_thread.start()

        print(f'{TAG} Device connected to {port}, waiting for initialization...')
        while self.busy:
            sleep(0.1)
        self.check_version()

        print(f'{TAG} Device initialized. Getting initial state...')
        self.heartbeat = time()  # last time device was alive
        self.update_state(initial=True)

        # start underflow check delay
        self.underflow_thread = threading.Thread(target=self._underflow_check)
        self.underflow_thread.daemon = True
        self.underflow_thread.start()

    def alive(self):
        """Check if the device is still alive within bound."""
        return time() - self.heartbeat < COMMUNICATION_TIMEOUT

    def ready(self):
        return not self.busy and self.alive() and not self.e_stop

    def outdated(self):
        return self.state.last_update < datetime.now() - timedelta(COMMUNICATION_TIMEOUT)

    def update_state(self, tube=None, quick=False, initial=False):
        """Get the last state of the device. If cached state is outdated, a new sensor reading is requested."""
        # Return cached state if not outdated nor unstable.
        if not initial and not self.state.dynamic and self.outdated():
            return self.state

        arg = ""

        if quick:
            arg += "2"
        else:
            arg += "1"

        if tube:
            arg += f" {tube};"
        else:
            arg += ";"

        # while busy, wait
        while not self.ready():
            sleep(1)

        # Ask for new state reading.
        size_buffer = self.last_printed_buf_line

        self.write(arg)
        self.busy = True

        # Wait for the state to be received.
        total_wait = 0
        while True:
            # wait for device to be ready again after requesting state
            while not self.ready():
                sleep(0.1)
                total_wait += 0.1

            # todo: not robust looking for {
            if self.last_printed_buf_line > size_buffer and self.read_buffer[-2][0] == '{':
                # If we received a line starting with {, we have received the new state.
                break

            if total_wait > COMMUNICATION_TIMEOUT and self.ready():
                raise RuntimeError(
                    f"Waiting too long for state to be communicated. {self.busy=}, {self.ready()=}")

        # New state retrieved, parse it.
        state = self.get_last_raw_state()
        if state:
            self.state = ClaireState(state)
            return self.state
        raise SensorError(f"Failed to retrieve new state, got {state}.")

    def _underflow_check(self):
        """
        Checks whether underflow occurs, i.e., the outflow pump is pumping while water is lower than
        lowest allowed level. This check prevents the outflow pump from pumping air.
        """
        TAG = "UNDERFLOW_CHECK"
        while True:
            # if state is stale or dynamic, update is required
            if self.state.dynamic or self.state.last_update < datetime.now() - timedelta(COMMUNICATION_TIMEOUT):
                print(f"{TAG} State is stale or dynamic, requiring fresh state. {self.state.last_update=}, {datetime.now()=}")

                # if not ready, delay and try again
                if not self.ready():
                    sleep(1)
                    continue

                # do quick update
                self.update_state(quick=True)

            # check underflows
            if self.state.Tube1_sonar_dist_mm > TUBE_MAX_LEVEL:
                # if outflow is active while inflow is stopped, error out
                if self.state.Tube1_outflow_duty > 0 and self.state.Tube1_inflow_duty == 0:
                    self.set_outflow(1, 0)
                    print(
                        f'{TAG}: WARN: Low water level detected in tube 1: {self.state.Tube1_sonar_dist_mm}. Stopped outflow')

            elif self.state.Tube2_sonar_dist_mm > TUBE_MAX_LEVEL:
                # if outflow is active while inflow is stopped, error out
                if self.state.Tube2_outflow_duty > 0 and self.state.Tube2_inflow_duty == 0:
                    self.set_outflow(2, 0)
                    print(
                        f'{TAG}: WARN: Low water level detected in tube 2: {self.state.Tube2_sonar_dist_mm}. Stopped outflow')

            else:
                if DEBUG:
                    print(f'{TAG}: No underflow detected in watchdog.')

            # Stop checking underflow.
            if self.stopped:
                break

            sleep(UNDERFLOW_CHECK_INTERVAL)

    def _read_lines(self):
        """Read lines from the serial port and add to the buffer in a thread to not block the main thread."""
        while True:
            buf = self.ser.readlines()
            if buf:
                self.heartbeat = time()
                new_lines = [line.decode('utf-8').rstrip() for line in buf]
                self.read_buffer.extend(new_lines)
                if IMMEDIATE_OUTPUT:
                    self.print_new_lines_buf()
                # Check whether the new lines contain the ready signal.
                for line in new_lines:
                    if line == CLAIRE_READY_SIGNAL:
                        self.busy = False

            # Stop reading lines.
            if self.stopped:
                break

    def buf_lines(self) -> list[str]:
        """
        Return the lines in the buffer.

        :return: The lines in the buffer."""
        return self.read_buffer[:]

    def buf_lines_from(self, start) -> list[str]:
        """
        Return the lines in the buffer from the provided start.

        :param: The start index to read the buffer.
        :return: The content of the buffer from start."""
        return self.read_buffer[start:]

    def last_buf_lines(self) -> list[str]:
        """
        Return the lines in the buffer from :attr:`self.last_printed_buf_line`.

        :return: The content of the buffer from :attr:`self.last_printed_buf_line`."""
        return self.buf_lines_from(self.last_printed_buf_line + 1)

    def print_buf(self):
        """Print the lines in the buffer in blue to differentiate from experiment output."""
        # print to stderr to get colour
        for line in self.buf_lines():
            ColorPrinting.print_blue(line)

    def print_last_line_buf(self):
        """Prints the last line in the buffer in blue to differentiate from experiment output."""
        # print to stderr to get colour
        ColorPrinting.print_blue(self.read_buffer[-1])

    def print_new_lines_buf(self):
        """
        Prints the new (not yet printed) lines in the buffer in blue to differentiate from experiment output.
        """
        # print to stderr to get colour
        if len(self.read_buffer) > self.last_printed_buf_line + 1:
            for line in self.last_buf_lines():
                ColorPrinting.print_blue(line)
            self.last_printed_buf_line = len(self.read_buffer) - 1

    def check_version(self):
        """Check the version of the software on the Arduino."""
        # Version number is on the first line, as that reads "Initialising CLAIRE water management <version>"
        if len(self.read_buffer) == 0:
            raise RuntimeError("No output received from device during version check.")
        line = self.read_buffer[0]
        words = line.split(' ')

        # Sanity checking
        assert words[0] == "Initialising"
        assert len(words) == 5

        # Check version
        assert words[-1] == CLAIRE_VERSION, f"The CLAIRE software on the Arduino is version {words[-1]}, while this " \
                                            f"Python script is constructed for version {CLAIRE_VERSION}."

        # check if device is ready
        assert self.ready()

    def get_last_raw_state(self):
        """Get the last raw state of the device without polling."""
        # take buf backwards and try to coerce every line into dict
        for line in reversed(self.buf_lines()):
            try:
                state = utils.parse_str_dict(line)
                return state
            except ValueError:
                pass

    def print_state(self):
        """Print state of the system."""
        # seconds since state was grabbed
        if self.state:
            old = datetime.now() - self.state.last_update
            print(f'{TAG} State ({old} old): {self.state}')
        else:
            print(f'{TAG} State: N/A')

    def write(self, data):
        """
        Write data to the serial port.

        :param data: The data to write to the serial port.
        """
        # Can only send new command if Arduino is not busy.
        assert not self.busy
        if DEBUG:
            print(f'{TAG} Writing command: {data}')
        self.ser.write(data.encode('utf-8'))
        sleep(0.1)  # Sleep slightly to take communication delays into account

    def close(self):
        """Close the serial connection."""
        self.stopped = True
        self.read_thread.join()  # Wait until read thread has been stopped.
        self.underflow_thread.join()  # Wait until read thread has been stopped.
        self.ser.close()

    def e_stop(self):
        """Emergency stop the device."""
        self.write("3;")
        # wait for device to be ready again after e_stopping
        while not self.ready():
            sleep(0.1)
        self.update_state(quick=True)
        assert not self.state.dynamic, "Device is still dynamic after emergency stop."
        self.e_stop = True

    def reset(self):
        """Reset the device and clears the emergency stop."""
        self.write("7;")
        # wait for device to be ready again after resuming
        while not self.ready():
            sleep(0.1)
        self.update_state(quick=True)
        assert not self.state.dynamic, "Device is dynamic after resetting emergency stop."
        self.e_stop = False

    def set_water_level(self, tube, level):
        """
        Set the water level in the selected tube to the provided height.

        :param tube: The tube to set the water level, should be value from [1,2].
        :param level: The desired water level, should be value from [0,TUBE_MAX_LEVEL].
        """
        assert tube == 1 or tube == 2
        assert 0 <= level <= TUBE_MAX_LEVEL
        while not self.ready():
            sleep(1)
        self.write(f"5 {tube} {self.state.convert_level_to_distance(level)};")
        self.busy = True
        self.state.make_dynamic()

    def set_inflow(self, tube, rate):
        """
        Set the inflow in the tube to the provided rate.

        :param tube: The tube to set the water level, should be value from [1,2].
        :param rate: The desired inflow rate, should be value from [0,100].
        """
        assert tube == 1 or tube == 2
        assert 0 <= rate <= 100
        while not self.ready():
            sleep(1)
        pump = (tube - 1) * 2 + 1
        self.write(f"4 {pump} {rate};")
        self.state.make_dynamic()

    def set_outflow(self, tube, rate):
        """
        Set the outflow in the tube to the provided rate.

        :param tube: The tube to set the water level, should be value from [1,2].
        :param rate: The desired outflow rate, should be value from [0,100].
        """
        assert tube == 1 or tube == 2
        assert 0 <= rate <= 100
        while not self.ready():
            sleep(1)
        pump = tube * 2
        self.write(f"4 {pump} {rate};")
        self.state.make_dynamic()

    def wait_until_free(self):
        """Wait until the device is free."""
        while True:
            if not self.busy:
                return
            sleep(1)
