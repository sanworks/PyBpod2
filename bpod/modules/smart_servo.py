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
from bpod.utils.reactive_list import ReactiveList
import time
import math

MENU_BYTE = 212  # A byte that precedes each USB serial command. This shields the device from lower ASCII interference
                 # e.g. from Linux modem-manager
HANDSHAKE_REQUEST_BYTE = 249  # A byte sent to request a handshake response
HANDSHAKE_REPLY_BYTE = 250
N_CHANNELS = 3  # Motor channels on the device
N_ADDR = 3  # Addresses per channel. Each address may have 1 motor.
N_DIO = 3  # Number of DIO channels on the device
MODEL_NUMBERS = [1200, 1190, 1090, 1060, 1070, 1080, 1220, 1210, 1240, 1230,
                 1160, 1120, 1130, 1020, 1030, 1100, 1110, 1010, 1000]
MODEL_NAMES = ['XL330-M288', 'XL330-M077', '2XL430-W250', 'XL430-W250',
               'XC430-T150/W150', 'XC430-T240/W240', 'XC330-T288', 'XC330-T181',
               'XC330-M288', 'XC330-M181', '2XC430-W250', 'XM540-W270', 'XM540-W150',
               'XM430-W350', 'XM430-W210', 'XH540-W270', 'XH540-W150', 'XH430-W210', 'XH430-W350']
VALID_PROGRAM_TYPES = ['velocity', 'current_limit']


class BpodSmartServo(object):
    def __init__(self, port_name):
        """  Initializes the BpodSmartServo object, opens the USB serial port and initializes the device
             Args:
                 port_name (string) The name of the USB serial port as known to the OS
                                    Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
        """

        # Connect to device
        self.port = ArCom(port_name, 480000000)
        self.port.write([MENU_BYTE, HANDSHAKE_REQUEST_BYTE], 'uint8')  # Handshake
        reply = self.port.read(1, 'uint8')
        if reply != HANDSHAKE_REPLY_BYTE:
            raise SmartServoError('Error: An incorrect handshake byte was received.')

        # Get module information
        self.port.write([MENU_BYTE, ord('?')], 'uint8')
        self._firmwareVersion = self.port.read(1, 'uint32', 'builtin')
        self._hardwareVersion = self.port.read(1, 'uint32', 'builtin')
        self._max_programs = self.port.read(1, 'uint32', 'builtin')
        self._max_steps = self.port.read(1, 'uint32', 'builtin')

        # Setup internal variables
        self._programLoaded = [False] * self._max_programs
        self._isConnected = [[False for col in range(N_ADDR)] for row in range(N_CHANNELS)]
        self._detectedModelName = [["" for col in range(N_ADDR)] for row in range(N_CHANNELS)]
        self.dioTargetProgram = [1] * N_DIO
        self.dioFallingEdgeOp = [0] * N_DIO
        self.dioRisingEdgeOp = [0] * N_DIO
        self.dioDebounce = [0.01] * N_DIO

        # Detect servos
        self.detect_motors()

    def stop(self, channel, address):
        """ stop() Stops a specific motor. Fails if not acknowledged. Torque does not need to be manually re-enabled.
            Args:
                channel (int) the target motor channel in range [1, N_CHANNELS]
                address (int) the target motor address on its channel in range [1, N_ADDR]
            Returns:
                none
        """
        self.port.write([MENU_BYTE, ord('X'), channel, address], 'uint8')
        self._confirm_transfer('stopping motor on channel: ' + str(channel) + ' address: ' + str(address))

    def STOP(self):  # Emergency stop
        # STOP() stops all motors by setting their torque to 0, and cancels any ongoing motor programs.
        # After an emergency stop, torque must be re-enabled by clearing the class instance
        # or by setting motorMode for each motor.

        self.port.write([MENU_BYTE, ord('!')], 'uint8')
        confirmed = self.port.read(1, 'uint8')
        if confirmed != 1:
            raise SmartServoError('***ALERT!*** Emergency stop not confirmed.')
        print('!! Emergency Stop Acknowledged !!')
        print('All motors now have torque disabled.')
        print('Re-enable motor torque by setting motorMode for each motor.')

    def detect_motors(self):
        # detect_motors returns information about any connected motors, and marks them available for use.

        print('Detecting motors...')
        self.port.write([MENU_BYTE, ord('D')], 'uint8')  # 'D' = Command to detect motors
        time.sleep(1)
        n_motors_found = math.floor(self.port.bytes_available() / 6)
        for i in range(n_motors_found):
            motor_channel = self.port.read(1, 'uint8', 'builtin')
            motor_address = self.port.read(1, 'uint8', 'builtin')
            motor_model = self.port.read(1, 'uint32', 'builtin')
            model_name = 'Unknown model'
            if motor_model in MODEL_NUMBERS:
                model_name = MODEL_NAMES[MODEL_NUMBERS.index(motor_model)]
            self._isConnected[motor_channel-1][motor_address-1] = True
            self._detectedModelName[motor_channel-1][motor_address-1] = model_name
            print('Found: Ch: ' + str(motor_channel) + ' Address: ' + str(motor_address) + ' Model: ' + model_name)

    def set_motor_address(self, channel, current_address, new_address):
        """ set_motor_address() Sets a motor's address on its channel. This is useful for setting up daisy-chain configs
            The address change is to the motor's EEPROM, and will persist across power cycles.
            Args:
                channel (int) the target motor channel in range [1, N_CHANNELS]
                current_address (int) the current target motor address on its channel in range [1, N_ADDR]
                new_address (int) the new target motor address on its channel in range [1, N_ADDR]
            Returns:
                none
        """
        # Confirm address availability
        if not self._isConnected[channel-1][current_address-1]:
            raise SmartServoError('Error setting motor address. No motor registered at channel: '
                                  + str(channel) + ' address: ' + str(current_address))
        if self._isConnected[channel-1][new_address-1]:
            raise SmartServoError('Error setting motor address. A motor is already registered at channel: '
                                  + str(channel) + ' address: ' + str(new_address))

        # Change the address
        self.port.write([MENU_BYTE, ord('I'), channel, current_address, new_address], 'uint8')
        self._confirm_transfer('setting motor address')
        self._isConnected[channel-1][current_address-1] = False
        print('Address Change Acknowledged.')
        print(' ')
        self.detect_motors()

    def new_program(self):
        """ new_program() returns a blank motor program for use with add_movement() and load_program()
            Args:
                none
            Returns:
                A blank motor program (dict)
        """
        program = {
            'nSteps': 0,
            'moveType': 'velocity',  # must be either 'velocity' or 'current_limit'
            'nLoops': 0,
            'channel': [0] * self._max_steps,
            'address': [0] * self._max_steps,
            'goalPosition': [0] * self._max_steps,
            'movementLimit': [0] * self._max_steps,
            'stepTime': [0] * self._max_steps
        }
        return program

    def add_movement(self, program, channel, address, goal_position, movement_limit, step_time):
        """
         add_movement() adds a movement to an existing motor program.

         Arguments:
         program: The program dict to be extended with a new step
         channel: The target motor's channel on the device, an integer in range [1, N_CHANNELS]
         address: The target motor's address on the target channel, an integer in range [1, N_ADDR]
         goal_position: The position the motor will move to on this step (units = degrees)
         ***Pass only if the program's moveType = 'velocity':
                  movement_limit: The maximum velocity of the movement (units = rev/s).
                  Use 0 for max velocity.
         ***Pass only if moveType = 'current_limit':
                  movement_limit: The maximum current draw for the movement (unit = mA)
         step_time: The time when this movement will begin with respect to motor program start (units = seconds)

        % Returns:
        % The original program dict, modified with the added step
        """
        if program['nSteps'] >= self._max_steps:
            raise ValueError('Exceeded the maximum number of steps')
        n_steps = program['nSteps']
        program['channel'][n_steps] = channel
        program['address'][n_steps] = address
        program['goalPosition'][n_steps] = goal_position
        program['movementLimit'][n_steps] = movement_limit
        program['stepTime'][n_steps] = step_time
        program['nSteps'] += 1
        return program

    def set_loop_duration(self, program, loop_duration):
        """
        set_loop_duration() sets a duration for which to loop an existing motor program.
        A looping program returns to time 0 each time it completes its sequence of steps,
        and continues looping the program until loopDuration seconds.

        Arguments:
        program: The program structure to be modified with the new loop duration
        loop_duration: The duration for which to loop the motor program each time it is run (units = seconds)
                      Use 0 for no looping.

        Returns:
        The original program dict, modified with the new loop duration
        """
        program['loopDuration'] = loop_duration
        return program

    def load_program(self, program_index, program):
        """
        % load_program() loads a motor program to the Smart Servo Module's memory.
        % The Smart Servo Module can store up to 100 programs of up to 256 steps each.
        %
        % Arguments:
        % program_index: The program's index on the device (integer in range 0-99)
        % program: The program dict to load to the device at position program_index
        """
        n_steps = program['nSteps']
        n_loops = program['nLoops']
        move_type = program['moveType']
        channel = program['channel'][:n_steps]
        address = program['address'][:n_steps]
        goal_position = program['goalPosition'][:n_steps]
        movement_limit = program['movementLimit'][:n_steps]
        step_time = [int(t * 1000) for t in program['stepTime'][:n_steps]]

        # Sort moves by timestamps if necessary
        if any(step_time[i] > step_time[i + 1] for i in range(len(step_time) - 1)):
            sorted_indices = sorted(range(len(step_time)), key=lambda k: step_time[k])
            channel = [channel[i] for i in sorted_indices]
            address = [address[i] for i in sorted_indices]
            goal_position = [goal_position[i] for i in sorted_indices]
            movement_limit = [movement_limit[i] for i in sorted_indices]
            step_time = [step_time[i] for i in sorted_indices]

        move_type_integer = VALID_PROGRAM_TYPES.index(move_type)

        self.port.write([MENU_BYTE, ord('L'), program_index, n_steps, move_type_integer], 'uint8',
                        channel, 'uint8', address, 'uint8', goal_position, 'float32', movement_limit, 'float32',
                        step_time, 'uint32', n_loops, 'uint32')
        self._confirm_transfer('loading motor program')
        self._programLoaded[program_index] = True

    def run_program(self, program_index):
        """
        runProgram() runs a previously loaded motor program. Programs can also be run directly from the
        state machine with the 'R' command (see 'Serial Interfaces' section on the Bpod wiki)

        Args:
            program_index: The index of the program to run (integer, range = 0-99)
        """
        if not self._programLoaded[program_index]:
            raise SmartServoError('Cannot run motor program ' + str(program_index)
                                  + ', it must be loaded to the device first. ')
        self.port.write([MENU_BYTE, ord('R'), program_index], 'uint8')

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8')
        if msg != 1:
            raise SmartServoError("Error " + transfer_description + ". Confirm code not returned.")

    def __del__(self):
        self.port.close()


class SmartServoError(Exception):
    pass
