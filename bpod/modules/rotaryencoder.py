"""
----------------------------------------------------------------------------

This file is part of the Sanworks PyBpod2 repository
Copyright (C) Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

# BpodRotaryEncoder is a class to control the Bpod Rotary Encoder Module via its USB connection to the PC.
# ---Usage example---
# from bpod.modules.rotaryencoder import BpodRotaryEncoder
# R = BpodRotaryEncoder('COM3') # Replace 'COM3' with the name of the module's USB serial port
# R.wrap_point = 180  # Return to the opposite end of the circular position range when the encoder reaches 180 degrees
# R.wrap_mode = 'bipolar' # Set the circular position range to +/- R.wrap_point: [-180, 180]
# R.send_threshold_events = False  # Do not send threshold events to the Bpod State Machine
# del R  # Delete the object

from bpod.utils.arcom import ArCom
from bpod.utils.reactive_list import ReactiveList

MAX_THRESHOLDS = 8  # Maximum number of thresholds that can be configured


class BpodRotaryEncoder(object):
    def __init__(self, port_name):
        """  Initializes the BpodValveDriver object and opens the USB serial port
             Args:
                 port_name (string) The name of the USB serial port as known to the OS
                                    Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
        """

        # Setup USB serial port and confirm connection
        self.port = ArCom(port_name, 12000000)

        self.port.write([ord('C'),ord('X')], 'uint8')  # Handshake + reset data streams
        reply = self.port.read(1, 'uint8')
        if reply != 217:
            raise RotaryEncoderError('Error: An incorrect handshake byte was received.')

        # Set internal parameters
        self._thresholds = ReactiveList([0] * MAX_THRESHOLDS, update_callback=self.threshold_updater)
        self._wrap_point = 180
        self._wrap_mode = 'bipolar'
        self._send_threshold_events = False
        self._module_output_stream = False
        self._module_stream_prefix = 'M'
        self._half_point = 512
        self._use_advanced_thresholds = False

        # Read hardware version
        self.port.write([ord('I'), ord('M')], 'uint8')
        reply = self.port.read(1, 'uint8')
        self.hardware_version = 1
        if reply == 0:  # 'I' command sets module stream prefix. Module stream is absent on v1 so reply = 0
            self.hardware_version = 2
            self._half_point = 2048
            self._use_advanced_thresholds = True
            self.port.set_baud(480000000)

        # Set default user parameters (synchronizes defaults to the device)
        self.set2default_parameters()

    @property
    def thresholds(self):
        return self._thresholds

    @thresholds.setter
    def thresholds(self, new_thresholds):
        """ Sets the thresholds by calling threshold_updater()
            Args:
                new_thresholds, a 1 x 8 list of ints indicating position thresholds for events. Units = degrees.
            Returns:
                none
        """
        # Sanitize new_thresholds:
        if len(new_thresholds) != MAX_THRESHOLDS:
            raise RotaryEncoderError('Error setting thresholds: threshold list length must equal' + str(MAX_THRESHOLDS))

        # Set thresholds
        self._thresholds = ReactiveList(new_thresholds, update_callback=self.threshold_updater)
        self.threshold_updater(new_thresholds)

    def threshold_updater(self, new_thresholds):
        """ Sets the position thresholds on the rotary encoder module
            Args:
                new_thresholds, a 1 x 8 list of ints. Units = degrees
            Returns:
                none
        """

        threshold_positions = self.degrees2pos(new_thresholds)
        self.port.write([ord('T'), MAX_THRESHOLDS], 'uint8', threshold_positions, 'int16')
        self._confirm_transfer('setting thresholds')

    @property
    def wrap_point(self):
        return self._wrap_point

    @wrap_point.setter
    def wrap_point(self, new_wrap_point):
        """ Sets wrap_point, the position at which the encoder wraps to
            the opposite end of its range of circular positions
            Args:
                new_wrap_point (float) The wrap position in degrees
            Returns:
                none
        """
        wrap_point_pulses = self.degrees2pos(new_wrap_point)
        self.port.write(ord('W'), 'uint8', wrap_point_pulses, 'int16')
        self._confirm_transfer('setting wrap point')
        self._wrap_point = new_wrap_point

    @property
    def wrap_mode(self):
        return self._wrap_mode

    @wrap_mode.setter
    def wrap_mode(self, new_wrap_mode):
        """ Sets wrap_mode, the relationship between wrap_point and the range of circular positions.
            Args:
                new_wrap_mode (str) The new mode: 'bipolar' for range = [-wrap_point, wrap_point]
                                                  'unipolar' for range = [0, wrap_point]
            Returns:
                none
        """
        valid_wrap_modes = ['bipolar', 'unipolar']
        if new_wrap_mode not in valid_wrap_modes:
            raise RotaryEncoderError('Invalid value for wrap_mode: Valid values are Bipolar or Unipolar.')
        self.port.write((ord('M'), valid_wrap_modes.index(new_wrap_mode)), 'uint8')
        self._confirm_transfer('setting wrap mode')
        self._wrap_mode = new_wrap_mode

    @property
    def send_threshold_events(self):
        return self._send_threshold_events

    @send_threshold_events.setter
    def send_threshold_events(self, new_state):
        """ Sets send_threshold_events, a flag true if threshold crossing events are returned to the Bpod State Machine
            Args:
                send_threshold_events (bool) True to send threshold events to the Bpod State Machine, False if not
            Returns:
                none
        """
        if not isinstance(new_state, bool):
            raise TypeError("The state of send_threshold_events must be a Boolean value: True or False")
        self.port.write((ord('V'), int(new_state)), 'uint8')
        self._confirm_transfer('setting send_threshold_events')
        self._send_threshold_events = new_state

    @property
    def module_output_stream(self):
        return self._module_output_stream

    @module_output_stream.setter
    def module_output_stream(self, new_state):
        """ Sets module_output_stream, a flag true if positions are streamed directly to another Bpod module
            via the rotary encoder module's 'Module' jack (Hardware v1 only). If enabled, positions are formatted as:
            [Prefix, Position1, Prefix, Position2, .... Prefix, PositionN] where Prefix is uint8 and Position is int16
            Args:
                new_state (bool) True to stream positions, False if not
            Returns:
                none
        """
        if not isinstance(new_state, bool):
            raise TypeError("The state of module_output_stream must be a Boolean value: True or False")
        self.port.write((ord('O'), int(new_state)), 'uint8')
        self._confirm_transfer('setting module_output_stream')
        self._module_output_stream = new_state

    @property
    def module_stream_prefix(self):
        return self._module_stream_prefix

    @module_stream_prefix.setter
    def module_stream_prefix(self, new_prefix):
        """ Sets module_stream_prefix, a byte that precedes each position streamed directly to another Bpod module
            via the rotary encoder module's 'Module' jack (Hardware v1 only). If enabled, positions are formatted as:
            [Prefix, Position1, Prefix, Position2, .... Prefix, PositionN] where Prefix is uint8 and Position is int16
            Args:
                new_prefix (uint8 or Char)
            Returns:
                none
        """
        if isinstance(new_prefix, str) and len(new_prefix) == 1:
            new_prefix = ord(new_prefix)
        if not isinstance(new_prefix, int):
            raise ValueError("Prefix value must be an integer")

        if not (0 <= new_prefix <= 255):
            raise ValueError("Prefix value must be in the range 0-255")

        self.port.write((ord('I'), new_prefix), 'uint8')
        self._confirm_transfer('setting module_stream_prefix')
        self._module_stream_prefix = new_prefix

    def zero_position(self):
        """ Resets the encoder position to 0, This can also be done directly from the State Machine with the 'Z' command
            Args:
                none
            Returns:
                none
        """
        self.port.write(ord('Z'), 'uint8')

    def set_position(self, pos_degrees):
        """ Sets the current encoder position to a new position.
            Args:
                pos_degrees (float) the desired position in degrees
            Returns:
                none
        """
        #Todo: sanitize pos_degrees, ensure it is within range defined by wrap_point and wrap_mode
        pos_pulses = self.degrees2pos(pos_degrees)
        self.port.write(ord('P'), 'uint8', pos_pulses, 'int16')
        self._confirm_transfer('setting encoder position')

    def enable_thresholds(self, thresholds):
        """ Enables threshold(s) indicated with a list of 1 (enable) or 0 (do not enable)
            Note that thresholds are disabled when crossed, and will not generate threshold events until re-enabled.
            Note that enabled thresholds will not be disabled by this function.
            Args:
                enable_thresholds (list of ints) A list indicating thresholds to enable. 1 to enable, 0 if not
            Returns:
                none
        """
        if not (1 <= len(thresholds) <= MAX_THRESHOLDS):
            raise ValueError("thresholds must be a list of 0 and 1s indicating which threshold(s) to enable")
        thresholds.reverse()
        binary_string = ''.join(str(bit) for bit in thresholds)
        binary_bits = int(binary_string, 2)
        self.port.write((ord(';'),  binary_bits), 'uint8')

    def set2default_parameters(self):
        """ Sets the BpodRotaryEncoder object (and connected device) to default parameters.
            Args:
                none
            Returns:
                none
        """
        self.wrap_point = 180
        self.wrap_mode = 'bipolar'
        self.send_threshold_events = False
        if self.hardware_version == 1:
            self.module_output_stream = False
            self.module_stream_prefix = 'M'

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8')
        if msg != 1:
            raise RotaryEncoderError("Error " + transfer_description + ". Confirm code not returned.")

    def degrees2pos(self, degrees):
        """ Converts degrees to rotary encoder positions.
            Args:
                degrees: A position or list of positions given in degrees
            Returns:
                pos: A position or list of positions given in pulses
        """
        if isinstance(degrees, list):
            # Perform the conversion for a list
            return [(i / 180) * self._half_point for i in degrees]
        else:
            # Perform the conversion for a single float value
            return (degrees / 180) * self._half_point

    def __repr__(self):
        """  Self-description when the object is entered into the Python shell
             with no properties or methods specified
        """
        return ('\nBpodRotaryEncoder with user properties:' + '\n\n'
                'Port: ' + self.port.serialObject.port + '\n'
                )

    def __del__(self):
        self.port.close()


class RotaryEncoderError(Exception):
    pass
