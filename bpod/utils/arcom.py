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

# ArCOM (Arduino Communication) wraps PySerial and numpy to streamline communication
# of numpy arrays to and from equivalent types on an Arduino-programmable microcontroller.
# A matching class for Arduino is provided in the ArCOM repository: https://github.com/sanworks/ArCOM

import numpy as np
import serial


class ArCom(object):
    def __init__(self, serial_port_name, baud_rate):
        """  Initializes the ArCOM object and opens the USB serial port
             Args:
                 serial_port_name (string) The name of the USB serial port as known to the OS
                                           Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
                 baud_rate (uint32) The speed of the target USB serial device (bits/second)
             Returns:
                 none
        """
        self._typeNames = ('uint8', 'int8', 'char', 'uint16', 'int16', 'uint32', 'int32', 'float32', 'float64')
        self._formatNames = ('numpy', 'builtin', 'builtin_list')
        self._typeBytes = (1, 1, 1, 2, 2, 4, 4, 4, 8)
        self._typeFormat = (0, 0, 0, 0, 0, 0, 0, 1, 1)  # Native type equivalent. 0 = int, 1 = float
        self.serial_port = serial.Serial(serial_port_name, baud_rate, timeout=10, rtscts=True)

    def close(self):
        """  Closes the USB serial port
             Args:
                 none
             Returns:
                 none
         """
        self.serial_port.close()

    def bytes_available(self):
        """  Returns the number of bytes available to read from the USB serial buffer
             Args:
                 none
             Returns:
                 nBytes (int) the number of bytes available to read
         """
        return self.serial_port.in_waiting

    def flush_input_buffer(self):
        """  Clears all bytes available to read from the USB serial buffer
             Args:
                 none
             Returns:
                 none
         """
        self.serial_port.reset_input_buffer()

    def set_baud(self, new_baud_rate):
        self.serial_port.baudrate = new_baud_rate

    def write(self, *arg):
        """  Write bytes to the USB serial buffer
             Args:
                 Arguments containing a message to write. The message format is:
                 # First value:
                     Arg 1. A single value or list of values to write
                     Arg 2. The datatype of the data in Arg 1 (must be supported, see self._typeBytes)
                 # Additional values (optional) given as pairs of arguments
                     Arg N. An additional value or list of values to write
                     Arg N+1. The datatype of arg N
             Returns:
                 none
         """
        n_types = int(len(arg)/2)
        arg_pos = 0
        message_bytes = b''
        for i in range(0, n_types):
            data = arg[arg_pos]
            arg_pos += 1
            datatype = arg[arg_pos]
            arg_pos += 1
            if (datatype in self._typeNames) is False:
                raise ArCOMError('Error: ' + datatype + ' is not a data type supported by ArCOM.')
            
            if type(data).__module__ == np.__name__:
                npdata = data.astype(datatype)
            else:
                npdata = np.array(data, dtype=datatype)
            message_bytes += npdata.tobytes()
        self.serial_port.write(message_bytes)

    def read(self, *arg):
        """  Read data from the USB serial buffer
             Args:
                 Arguments containing value(s) to read. The message format is:
                     Arg 0. (int) The number of values to read
                     Arg 1. (char) The numpy datatype of the data in Arg 0 (must be supported, see self._typeBytes)
                     Arg 2 (optional, string): The output format. 'numpy': numpy array (default)
                                                                  'builtin': int or float (single values only)
                                                                  'builtin_list': list of int or float
             Returns:
                 The data requested, returned as a numpy ndarray, int or float
         """
        num_values = arg[0]
        datatype = arg[1]
        output_format = 0  # 0 = numpy array, 1 = builtin int() or float(), 2 = list of int() or float()
        if len(arg) > 2:
            if arg[2] == self._formatNames[1]:
                if num_values > 1:
                    raise ArCOMError('Error: Builtin int or float format can only be used for reading single values.')
                output_format = 1
            elif arg[2] == self._formatNames[2]:
                output_format = 2
        if (datatype in self._typeNames) is False:
            raise ArCOMError('Error: ' + datatype + ' is not a data type supported by ArCOM.')
        type_index = self._typeNames.index(datatype)
        byte_width = self._typeBytes[type_index]
        n_bytes2read = num_values*byte_width
        message_bytes = self.serial_port.read(n_bytes2read)
        n_bytes_read = len(message_bytes)
        if n_bytes_read < n_bytes2read:
            raise ArCOMError('Error: serial port timed out. ' + str(n_bytes_read) +
                             ' bytes read. Expected ' + str(n_bytes2read) + ' byte(s).')
        output = np.frombuffer(message_bytes, datatype)
        if output_format > 0:  # Convert from numpy to int or float
            if self._typeFormat[type_index] == 0:
                output = output.astype(int)
            elif self._typeFormat[type_index] == 1:
                output = output.astype(float)
            if output_format == 1:
                output = output[0]
        return output

    def __del__(self):
        self.serial_port.close()


class ArCOMError(Exception):
    pass
