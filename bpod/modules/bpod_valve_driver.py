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

from bpod.utils.arcom import ArCom


class BpodValveDriver(object):
    def __init__(self, port_name):
        """  Initializes the BpodValveDriver object and opens the USB serial port
             Args:
                 port_name (string) The name of the USB serial port as known to the OS
                                    Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
        """
        self.Port = ArCom(port_name, 12000000)
        self.Port.write(255, 'uint8')  # Handshake
        reply = self.Port.read(1, 'uint8')
        if reply != 254:
            raise ValveModuleError('Error: An incorrect handshake byte was received.')
        self.firmwareVersion = self.Port.read(1, 'uint8')
        self.Port.write((ord('B'), 0), 'uint8')  # Close any open valves
        confirmed = self.Port.read(1, 'uint8')
        self._valveStates = [0]*8

    def set_valve(self, valve_id, valve_state):
        """  Sets the state of a valve by index
             Args:
                 valve_id (uint8) The index of the target valve
                 valve_state (uint8) The new state of the valve (1 = open, 0 = closed)
             Returns:
                 none
         """
        if (valve_id < 1) or (valve_id > 8):
            raise ValveModuleError('Error: Invalid valveID. Value must be in range: [1,8]')
        if valve_state not in [0, 1]:
            raise ValveModuleError('Error: Invalid valveState. Value must be in range: [0,1]')
        new_state = self._valveStates
        new_state[valve_id - 1] = valve_state
        valve_bits = 0
        for i in range(8):
            valve_bits += (new_state[i] * (2 ** i))
        self.set_valve_bits(valve_bits)

    def set_valve_bits(self, valve_bits):
        """  Sets the state of multiple valves as bits of a byte
                     Args:
                         valve_bits (uint8) A byte whose bits indicate the state of the valves
                                            1 = open, 0 = closed
                     Returns:
                         none
        """
        if (valve_bits < 0) or (valve_bits > 255):
            raise ValveModuleError('Error: Invalid valveID. Value must be in range: [0,255]')
        self.Port.write((ord('B'), valve_bits), 'uint8')
        confirmed = self.Port.read(1, 'uint8')
        new_state = [0]*8
        for i in range(8):
            new_state[i] = (valve_bits >> i) & 1
        self._valveStates = new_state

    def __repr__(self):
        """  Self-description when the object is entered into the Python shell
             with no properties or methods specified
        """
        return ('\nBpodValveDriver with user properties:' + '\n\n'
                'Port: ArCOMObject(' + self.Port.serialObject.port + ')' + '\n'
                'firmwareVersion: ' + str(self.firmwareVersion) + '\n'
                '_valveState: ' + str(self._valveStates) + '\n'
                )

    def __del__(self):
        self.Port = []


class ValveModuleError(Exception):
    pass
