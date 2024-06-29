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
        self.motor = None
        self.port = ArCom(port_name, 480000000)
        self.port.write([MENU_BYTE, HANDSHAKE_REQUEST_BYTE], 'uint8')  # Handshake
        reply = self.port.read(1, 'uint8')
        if reply != HANDSHAKE_REPLY_BYTE:
            raise RuntimeError('Error: An incorrect handshake byte was received.')

        # Get module information
        self.port.write([MENU_BYTE, ord('?')], 'uint8')
        self._firmware_version = self.port.read(1, 'uint32', 'builtin')
        self._hardware_version = self.port.read(1, 'uint32', 'builtin')
        self._max_programs = self.port.read(1, 'uint32', 'builtin')
        self._max_steps = self.port.read(1, 'uint32', 'builtin')

        # Setup internal variables
        self._program_loaded = [False] * self._max_programs
        self._is_connected = [[False for col in range(N_ADDR)] for row in range(N_CHANNELS)]
        self._detected_model_name = [["" for col in range(N_ADDR)] for row in range(N_CHANNELS)]
        self.dio_target_program = [1] * N_DIO
        self.dio_falling_edge_op = [0] * N_DIO
        self.dio_rising_edge_op = [0] * N_DIO
        self.dio_debounce = [0.01] * N_DIO

        # Detect and initialize servos
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
        """ STOP() stops all motors by setting their torque to 0, and cancels any ongoing motor programs.
            After an emergency stop, torque must be re-enabled by clearing the class instance
            or by setting BpodSmartServo.motor[chan][addr].control_mode for each motor.
        """

        self.port.write([MENU_BYTE, ord('!')], 'uint8')
        confirmed = self.port.read(1, 'uint8', 'builtin')
        if confirmed != 1:
            raise RuntimeError('***ALERT!*** Emergency stop not confirmed.')
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
            self._is_connected[motor_channel-1][motor_address-1] = True
            self._detected_model_name[motor_channel-1][motor_address-1] = model_name
            print('Found: Ch: ' + str(motor_channel) + ' Address: ' + str(motor_address) + ' Model: ' + model_name)

        # Create instances of SmartServoInterface class (below) for each motor
        self.motor = [[SmartServoInterface(self.port, x, y, -1) for y in range(N_ADDR+1)] for x in range(N_CHANNELS+1)]
        for chan in range(N_CHANNELS):
            for addr in range(N_ADDR):
                if self._is_connected[chan][addr]:
                    self.motor[chan+1][addr+1] = SmartServoInterface(self.port, chan + 1, addr + 1,
                                                                     self._detected_model_name[chan][addr])

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
        if not self._is_connected[channel-1][current_address-1]:
            raise RuntimeError('Error setting motor address. No motor registered at channel: '
                                  + str(channel) + ' address: ' + str(current_address))
        if self._is_connected[channel-1][new_address-1]:
            raise RuntimeError('Error setting motor address. A motor is already registered at channel: '
                                  + str(channel) + ' address: ' + str(new_address))

        # Change the address
        self.port.write([MENU_BYTE, ord('I'), channel, current_address, new_address], 'uint8')
        self._confirm_transfer('setting motor address')
        self._is_connected[channel-1][current_address-1] = False
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
        self._program_loaded[program_index] = True

    def run_program(self, program_index):
        """
        runProgram() runs a previously loaded motor program. Programs can also be run directly from the
        state machine with the 'R' command (see 'Serial Interfaces' section on the Bpod wiki)

        Args:
            program_index: The index of the program to run (integer, range = 0-99)
        """
        if not self._program_loaded[program_index]:
            raise RuntimeError('Cannot run motor program ' + str(program_index)
                                  + ', it must be loaded to the device first. ')
        self.port.write([MENU_BYTE, ord('R'), program_index], 'uint8')

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8', 'builtin')
        if msg != 1:
            raise RuntimeError("Error " + transfer_description + ". Confirm code not returned.")

    def __del__(self):
        self.port.close()


class SmartServoInterface:

    def __init__(self, port, channel, address, model_name):
        """  Initializes a SmartServoInterface object, to configure and control an individual motor.
             Args:
                 port: the 'port' property of the associated BpodSmartServo object
                 channel (int): the channel on the device where the target motor is located
                 address (int): the address on the channel where the target motor is located
                 model_name (string): The model name of the motor. Must match a supported motor name in MODEL_NAMES
            Note that an instance of SmartServoInterface for each connected motor is automatically initialized at:
            BpodSmartServo.motor[channel][address]
        """
        self.live_instance = False
        self.channel = channel
        self.address = address
        self.port = None
        self.info = {}
        self._control_mode = 1
        self.motor_mode_ranges = [
            [0, 360],
            [-91800, 91800],
            [0, 360],
            [0, 0],
            [-91800, 91800]
        ]
        self.selected_mode_range = []
        self.ctrl_table = None

        if model_name != -1:
            self.ctrl_table = self._setup_control_table()
            self.port = port
            self.info['model_name'] = model_name
            self.live_instance = True
            if model_name:
                self.control_mode = 1
                self.set_max_velocity(0)
                self.set_max_acceleration(0)
                self.info['firmware_version'] = self.read_control_table(self.ctrl_table['FIRMWARE_VERSION'])

    @property
    def control_mode(self):
        return self._control_mode

    @control_mode.setter
    def control_mode(self, new_mode):
        """ Sets the control mode
            Args:
                new_mode (int), the new control mode. Valid modes are:
                                1 = Position. 2 = Extended Position. 3 = Current-limited position 4 = Speed 5 = Step
            Returns:
                none
        """
        self.assert_live_instance()
        if new_mode not in range(1, 6):
            raise ValueError('Invalid control mode. Valid modes are integers in range 1-5.')
        self.port.write([MENU_BYTE, ord('M'), self.channel, self.address, new_mode], 'uint8')
        self.confirm_transmission('setting mode')
        self._control_mode = new_mode
        self.selected_mode_range = self.motor_mode_ranges[new_mode - 1]

    def STOP(self):
        """ EMERGENCY STOP
            This function stops all motors by setting their torque to 0.
            It also stops any ongoing motor programs.
            Torque may be re-enabled by setting BpodSmartServo.motor[chan][addr].control_mode for each motor.
        """
        self.port.write([MENU_BYTE, ord('!')], 'uint8')
        confirmed = self.port.read(1, 'uint8')
        if confirmed != 1:
            raise RuntimeError('***ALERT!*** Emergency stop not confirmed.')
        print('!! Emergency Stop Acknowledged !!')
        print('All motors now have torque disabled.')
        print('Re-enable motor torque by setting BpodSmartServo.motor[chan][addr].control_mode for each motor.')

    def stop(self):
        """ % Stop the motor (non-emergency)"""
        self.port.write([MENU_BYTE, ord('X'), self.channel, self.address], 'uint8')
        self.confirm_transmission(f'stopping motor on channel: {self.channel} address: {self.address}')

    def set_max_velocity(self, max_velocity):
        """
        set_max_velocity() Sets the maximum velocity for all subsequent movements with setPosition(). Units = rev/s

        Args:
            max_velocity (float) the maximum movement velocity in rev/s
        """
        self.assert_live_instance()
        self.port.write([MENU_BYTE, ord('['), self.channel, self.address], 'uint8', max_velocity, 'float32')
        self.confirm_transmission('setting max velocity')

    def set_max_acceleration(self, max_acceleration):
        """
        set_max_acceleration() Sets the maximum acceleration for all subsequent movements with setPosition().

        Args:
            max_acceleration (float) the maximum acceleration in rev/s^2
        """
        self.assert_live_instance()
        self.port.write([MENU_BYTE, ord(']'), self.channel, self.address], 'uint8', max_acceleration, 'float32')
        self.confirm_transmission('setting max acceleration')

    def set_position(self, new_position, *args):
        """
        set_position() sets the position of the motor shaft, in degrees of rotation.
        This method can only be used in control_mode 1 or 2.

        Required Arguments:
        new_position: The target position (units = degrees, range = 0-360)

        Optional Arguments (must both be passed in the following order):
        max_velocity (float): Maximum velocity enroute to target position (units = rev/min, 0 = Max)
        max_acceleration (float): Maximum acceleration enroute to target position (units = rev/min^2, 0 = Max)
        blocking (int): 1: Block the MATLAB command prompt until move is complete. 0: Don't.
        Note: If provided, max velocity and acceleration become the new settings for future movements.
        """
        self.assert_live_instance()
        if self.control_mode not in [1, 2]:
            raise ValueError(f'Motor {self.address} on channel {self.channel} must be in a position mode '
                             f'(modes 1 or 2) before calling set_position().')
        if not self.selected_mode_range[0] <= new_position <= self.selected_mode_range[1]:
            raise ValueError(f'Position goal {new_position} out of range. The target motor is in mode '
                             f'{self.control_mode}, with a position range of {self.selected_mode_range[0]} to '
                             f'{self.selected_mode_range[1]} degrees.')

        pos_bytes = new_position.to_bytes(4, 'little', signed=False)
        is_position_only = len(args) == 0
        if not is_position_only:
            max_velocity = args[0]
        if len(args) > 1:
            max_accel = args[1]
        blocking = args[2] if len(args) > 2 else 0

        if is_position_only:
            self.port.write([MENU_BYTE, ord('P'), self.channel, self.address], 'uint8', new_position, 'float32')
        else:
            self.port.write([MENU_BYTE, ord('G'), self.channel, self.address, blocking], 'uint8',
                            [new_position, max_velocity, max_accel], 'float32')

        self.confirm_transmission('setting position')
        if blocking:
            while self.port.bytes_available == 0:
                pass
            movement_complete = self.port.read(1, 'uint8')
            if movement_complete != 1:
                raise RuntimeError('Error setting position. Movement end acknowledgement not returned.')

    def set_current_limited_pos(self, new_position, max_current):
        """
        set_current_limited_pos() moves to a target position while drawing at most max_current milliamps of current.
        Setting a current limit controls the maximum force the motor will output enroute to the goal position,
        for applications where too much force could cause damage.
        This method can only be used in control_mode 3.

        Arguments:
        new_position (float): The target position. Units = Degrees
        max_current (float): The maximum current. Units = mA
        """
        self.assert_live_instance()
        if self.control_mode != 3:
            raise ValueError(f'Motor {self.address} on channel {self.channel} must be in current-limited '
                             f'position mode (mode 3) before calling set_current_limited_pos().')

        self.port.write([MENU_BYTE, ord('C'), self.channel, self.address], 'uint8',
                        [new_position, max_current], 'float32')
        self.confirm_transmission('setting position')

    def set_speed(self, new_speed):
        """
        set_speed() sets the motor's rotational velocity target for continuous rotation.
        This method can only be used in control_mode 4.

        Arguments:
            new_speed (float): The target speed. Units = rev/s
        """
        self.assert_live_instance()
        if self.control_mode != 4:
            raise ValueError(f'Motor {self.address} on channel {self.channel} '
                             f'must be in Speed mode (mode 4) before calling set_speed().')

        self.port.write([MENU_BYTE, ord('V'), self.channel, self.address], 'uint8', new_speed, 'float32')
        self.confirm_transmission('setting motor speed')

    def step(self, step_size_degrees):
        """
        step() Rotates by a fixed distance in degrees (+/-) relative to current shaft position
        This method can only be used in control_mode 5.

        Arguments:
            step_size_degrees (float): The amount to rotate. Units = degrees
        """
        self.assert_live_instance()
        if self.control_mode != 5:
            raise ValueError(f'Motor {self.address} on channel {self.channel} '
                             f'must be in step mode (mode 5) before calling step().')

        self.port.write([MENU_BYTE, ord('S'), self.channel, self.address], 'uint8', step_size_degrees, 'float32')
        self.confirm_transmission('stepping the motor')

    def get_position(self):
        """
        get_position() returns the current shaft position

        Arguments:
            None

        Returns:
            pos (float), the current position. Units = degrees
        """
        self.assert_live_instance()
        self.port.write([MENU_BYTE, ord('%'), self.channel, self.address], 'uint8')
        pos = self.port.read(1, 'float32', 'builtin')
        return pos

    def get_temperature(self):
        """
        get_temperature() returns the motor temperature

        Arguments:
            None

        Returns:
            temp (float), the current motor temperature. Units = degrees Celsius
        """
        self.assert_live_instance()
        return self.read_control_table(self.ctrl_table['PRESENT_TEMPERATURE'])

    def read_control_table(self, table_address):
        """
        read_control_table() is a low-level function to read a value from the motor's control table.
        NOTE: The control table indexes listed in _setup_control_table() are different from the control
              table addresses in a specific motor's datasheet.
              The Dynamixel2Arduino library converts these to the appropriate address for each motor.

        Arguments:
            table_address (int) the control table address to read from

        Returns:
            control_table_value (int32), the value read from the control table at address: table_address
        """
        self.port.write([MENU_BYTE, ord('T'), self.channel, self.address, table_address], 'uint8')
        value = self.port.read(1, 'int32', 'builtin')
        return value

    def _setup_control_table(self):
        # Returns the Dynamixel2Arduino control table as a dict
        ctrl_table = {
            'MODEL_NUMBER': 0,
            'MODEL_INFORMATION': 1,
            'FIRMWARE_VERSION': 2,
            'PROTOCOL_VERSION': 3,
            'ID': 4,
            'SECONDARY_ID': 5,
            'BAUD_RATE': 6,
            'DRIVE_MODE': 7,
            'CONTROL_MODE': 8,
            'OPERATING_MODE': 9,
            'CW_ANGLE_LIMIT': 10,
            'CCW_ANGLE_LIMIT': 11,
            'TEMPERATURE_LIMIT': 12,
            'MIN_VOLTAGE_LIMIT': 13,
            'MAX_VOLTAGE_LIMIT': 14,
            'PWM_LIMIT': 15,
            'CURRENT_LIMIT': 16,
            'VELOCITY_LIMIT': 17,
            'MAX_POSITION_LIMIT': 18,
            'MIN_POSITION_LIMIT': 19,
            'ACCELERATION_LIMIT': 20,
            'MAX_TORQUE': 21,
            'HOMING_OFFSET': 22,
            'MOVING_THRESHOLD': 23,
            'MULTI_TURN_OFFSET': 24,
            'RESOLUTION_DIVIDER': 25,
            'EXTERNAL_PORT_MODE_1': 26,
            'EXTERNAL_PORT_MODE_2': 27,
            'EXTERNAL_PORT_MODE_3': 28,
            'EXTERNAL_PORT_MODE_4': 29,
            'STATUS_RETURN_LEVEL': 30,
            'RETURN_DELAY_TIME': 31,
            'ALARM_LED': 32,
            'SHUTDOWN': 33,
            'TORQUE_ENABLE': 34,
            'LED': 35,
            'LED_RED': 36,
            'LED_GREEN': 37,
            'LED_BLUE': 38,
            'REGISTERED_INSTRUCTION': 39,
            'HARDWARE_ERROR_STATUS': 40,
            'VELOCITY_P_GAIN': 41,
            'VELOCITY_I_GAIN': 42,
            'POSITION_P_GAIN': 43,
            'POSITION_I_GAIN': 44,
            'POSITION_D_GAIN': 45,
            'FEEDFORWARD_1ST_GAIN': 46,
            'FEEDFORWARD_2ND_GAIN': 47,
            'P_GAIN': 48,
            'I_GAIN': 49,
            'D_GAIN': 50,
            'CW_COMPLIANCE_MARGIN': 51,
            'CCW_COMPLIANCE_MARGIN': 52,
            'CW_COMPLIANCE_SLOPE': 53,
            'CCW_COMPLIANCE_SLOPE': 54,
            'GOAL_PWM': 55,
            'GOAL_TORQUE': 56,
            'GOAL_CURRENT': 57,
            'GOAL_POSITION': 58,
            'GOAL_VELOCITY': 59,
            'GOAL_ACCELERATION': 60,
            'MOVING_SPEED': 61,
            'PRESENT_PWM': 62,
            'PRESENT_LOAD': 63,
            'PRESENT_SPEED': 64,
            'PRESENT_CURRENT': 65,
            'PRESENT_POSITION': 66,
            'PRESENT_VELOCITY': 67,
            'PRESENT_VOLTAGE': 68,
            'PRESENT_TEMPERATURE': 69,
            'TORQUE_LIMIT': 70,
            'REGISTERED': 71,
            'MOVING': 72,
            'LOCK': 73,
            'PUNCH': 74,
            'CURRENT': 75,
            'SENSED_CURRENT': 76,
            'REALTIME_TICK': 77,
            'TORQUE_CTRL_MODE_ENABLE': 78,
            'BUS_WATCHDOG': 79,
            'PROFILE_ACCELERATION': 80,
            'PROFILE_VELOCITY': 81,
            'MOVING_STATUS': 82,
            'VELOCITY_TRAJECTORY': 83,
            'POSITION_TRAJECTORY': 84,
            'PRESENT_INPUT_VOLTAGE': 85,
        }
        return ctrl_table

    def assert_live_instance(self):
        # Throws an error if the class instance is a placeholder for a motor not detected (e.g. in BpodSmartServo.motor)
        if not self.live_instance:
            raise RuntimeError(f'Motor not registered at channel: {self.channel}, address: {self.address}\n'
                               f'If a new servo was recently connected, run SmartServoModule.detectMotors().')

    def confirm_transmission(self, op_name):
        """
        confirm_transmission() reads the op confirmation byte, and errors out if a confirmation code was not returned
        """
        confirmed = self.port.read(1, 'uint8')
        if confirmed == 0:
            raise RuntimeError(f'Error {op_name}: the module denied your request.')
        elif confirmed != 1:
            raise RuntimeError(f'Error {op_name}: module did not acknowledge the operation.')

    def __del__(self):
        self.port = None

