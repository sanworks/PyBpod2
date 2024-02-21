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

# BpodValveDriver is a class to control the Bpod Valve Driver Module via its USB connection to the PC.
# ---Usage example---
# from bpod.modules.valvedriver import BpodValveDriver
# v = BpodValveDriver('COM3') # Replace 'COM3' with the name of the module's USB serial port
# The simplest way to set the valve state is using v.is_open:
# v.is_open[0] = 1  # Open valve 1 (as labeled on the device)
# v.is_open[3] = 0  # Close valve 4
# v.is_open = [0,0,0,1,0,1,1,0]  # Set valves 2,3,and 5 open, and all others closed.
# Another way to set the valves is to use bits of a byte.
# v.set_valve_bits(3)  # Set valves using bits of a byte to indicate open and closed valves. 3 = 00000011. This will
                       # set valves 1+2 open and all others closed.
# v.close_all()  # Close all valves
# del v  # Delete the object

from bpod.utils.arcom import ArCom
from bpod.utils.reactive_list import ReactiveList

MIN_FIRMWARE = 2  # Minimum firmware version compatible with this class
N_CHANNELS = 8


class BpodValveDriver(object):
    def __init__(self, port_name):
        """  Initializes the BpodValveDriver object and opens the USB serial port
             Args:
                 port_name (string) The name of the USB serial port as known to the OS
                                    Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
        """

        # Setup USB serial port and confirm connection
        self.port = ArCom(port_name, 12000000)
        self.port.write(255, 'uint8')  # Handshake
        reply = self.port.read(1, 'uint8')
        if reply != 254:
            raise ValveModuleError('Error: An incorrect handshake byte was received.')

        # Confirm firmware
        fv = self.port.read(1, 'uint32')
        fv = fv.astype(int)[0]  # From np to int (todo: replace this in ArCOM with an output format arg)
        if fv < MIN_FIRMWARE:
            raise ValveModuleError('Error: Old firmware detected. Please update to v' + str(MIN_FIRMWARE))
        self.firmware_version = fv

        # Set initial state
        self._is_open = ReactiveList([0]*N_CHANNELS, update_callback=self.is_open_updater)
        self.close_all()  # Close any open valves

    @property
    def is_open(self):
        return self._is_open

    @is_open.setter
    def is_open(self, new_state):
        """ Sets the state of is_open by calling is_open_updater()
            Args:
                new_state, a 1 x N_CHANNELS list of ints. 1 = open, 0 = closed
            Returns:
                none
        """
        # Sanitize new_state:
        if len(new_state) != N_CHANNELS:
            raise ValveModuleError('Error setting is_open: the state-list length must equal the number of valves.')
        if any(element not in [0, 1] for element in new_state):
            raise ValveModuleError('Error setting is_open: all valve states must be 0 (closed) or 1 (open).')

        # Set valve state
        self._is_open = ReactiveList(new_state, update_callback=self.is_open_updater)
        self.is_open_updater(new_state)

    def is_open_updater(self, new_state):
        """ Sets the state of all valves on the valve driver module
            Args:
                new_state, a 1 x nChannels list of ints. 1 = open, 0 = closed
            Returns:
                none
        """
        new_state.reverse()
        binary_string = ''.join(str(bit) for bit in new_state)
        integer_value = int(binary_string, 2)
        self.set_valve_bits(integer_value)

    def set_valve_bits(self, valve_bits):
        """  Sets the state of multiple valves as bits of a byte
                     Args:
                         valve_bits (int)   An int in range [0, 255] whose bits indicate the state of the valves
                                            1 = open, 0 = closed
                     Returns:
                         none
        """
        if (valve_bits < 0) or (valve_bits > 255):
            raise ValveModuleError('Error: Invalid valve_bits. Value must be in range: [0,255]')
        self.port.write((ord('B'), valve_bits), 'uint8')
        self._confirm_transfer('setting valve state')
        new_state = [0]*N_CHANNELS
        for i in range(N_CHANNELS):
            new_state[i] = (valve_bits >> i) & 1
        self._is_open = ReactiveList(new_state, update_callback=self.is_open_updater)

    def close_all(self):
        self.set_valve_bits(0)

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8')
        if msg != 1:
            raise ValveModuleError("Error " + transfer_description + ". Confirm code not returned.")

    def __repr__(self):
        """  Self-description when the object is entered into the Python shell
             with no properties or methods specified
        """
        return ('\nBpodValveDriver with user properties:' + '\n\n'
                'Port: ' + self.port.serialObject.port + '\n'
                'firmware_version: ' + str(self.firmware_version) + '\n'
                'is_open: ' + str(self._is_open) + '\n'
                )

    def __del__(self):
        self.port.close()


class ValveModuleError(Exception):
    pass
