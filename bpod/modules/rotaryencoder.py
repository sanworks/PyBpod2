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
# r = BpodRotaryEncoder('COM3') # Replace 'COM3' with the name of the module's USB serial port

# del r  # Delete the object

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

        # Set default parameters
        self._thresholds = ReactiveList([0]*MAX_THRESHOLDS, update_callback=self.threshold_updater)
        self.wrap_point = 180
        self.wrap_mode = 'bipolar'
        self.send_threshold_events = False
        self.module_output_stream = False
        self.module_stream_prefix = False

        # Set private parameters
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

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8')
        if msg != 1:
            raise RotaryEncoderError("Error " + transfer_description + ". Confirm code not returned.")

    def degrees2pos(self, degrees):
        return [(i/180)*self._half_point for i in degrees]

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
