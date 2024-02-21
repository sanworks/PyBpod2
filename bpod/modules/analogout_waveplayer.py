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

# BpodWavePlayer is a class to control the Bpod Analog Output Module running WavePlayer firmware.
# WavePlayer hosts a library of up to 64 sampled waveforms, and plays them back from any output channels on trigger.
#
# ---Usage example---
# #Imports
# from bpod.modules.analogout_waveplayer import BpodWavePlayer
# import numpy as np
# import time
#
# #Wave parameters:
# SF = 100000   # Sampling rate
# Freq = 1000   # Waveform frequency
# Duration = 1  # Waveform duration (interrupted by stop command below)
#
# #Setup device
# W = BpodWavePlayer('COM3')  # Replace 'COM3' with the WavePlayer's USB serial port
# W.sampling_rate = SF        # Set the sampling rate to 100kHz
# W.output_range = '-10V:10V' # Set the output range to [-10V, 10V]
# W.loop_mode[2] = True       # Enable loop mode on channel 3 (2 due to python 0-indexing)
# W.loop_duration[2] = 3      # Set channel 3 to loop the waveform for 3 seconds on each trigger
#
# #Create and load waveform
# Wave = np.sin(2 * np.pi * Freq * np.arange(Duration * SF) / SF)
# W.load(1, Wave)  # Load waveform to device memory at position 1
#
# #Playback and stop commands
# W.play([0, 1], 1)  # Play waveform 1 on channels 1 and 2
# time.sleep(0.5)  # Wait for 0.5s
# W.stop()  # Stop all playback
#
# del W  # Delete the object
#
# Notice: Differences from MATLAB BpodWavePlayer class:
# 1. Trigger Profiles are now obsolete and were not implemented. See play() method syntax.
#    Trigger separate waveforms on multiple channels from the state machine using the '>' command.
# 2. Python uses 0-indexing, so channels 1-4 on the device are targeted by this class as channels as 0-3

from bpod.utils.arcom import ArCom
from bpod.utils.reactive_list import ReactiveList
from math import ceil


class BpodWavePlayer(object):
    def __init__(self, port_name):
        """  Initializes the BpodWavePlayer object, opens the USB serial port and configures the device
             Args:
                 port_name (string) The name of the USB serial port as known to the OS
                                    Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
        """
        MIN_FIRMWARE = 6  # Minimum firmware version compatible with this class

        # Initialize USB connection
        self.port = ArCom(port_name, 12000000)  # Create ArCOM serial port

        # Exchange handshake
        self.port.write(227, 'uint8')  # Send handshake byte
        handshake = self.port.read(1, 'uint8')  # Read reply
        if handshake != 228:
            raise WavePlayerError('Error connecting to WavePlayer module. Incorrect handshake returned.')

        # Read firmware and hardware info
        self._info = dict(firmwareVersion=None, hardwareVersion=None, circuitRevision=None,
                          nChannels=0, maxWaveforms=0, maxTriggerProfiles=0)
        firmware_version = self.port.read(1, 'uint32').astype(int)[0]  # Read firmware version
        if firmware_version < MIN_FIRMWARE:
            raise WavePlayerError(f"WavePlayer module firmware is old. v{firmware_version} detected, v{MIN_FIRMWARE} required.")
        self._info["firmwareVersion"] = firmware_version
        self.port.write(ord('H'), 'uint8')  # Request hardware version info
        self._info["hardwareVersion"] = self.port.read(1, 'uint8').astype(int)[0]  # Read major hardware version
        self._info["circuitRevision"] = self.port.read(1, 'uint8').astype(int)[0]  # Read circuit revision
        self._info["outputRangeList"] = ['0V:5V', '0V:10V', '0V:12V', '-5V:5V', '-10V:10V', '-12V:12V']
        self._info["triggerModeList"] = ['Normal', 'Master', 'Toggle']
        self._info["outputRangeLimits"] = [[0, 5], [0, 10], [0, 12], [-5, 5], [-10, 10], [-12, 12]]
        self._info["maxSamplingRate"] = 100000
        if self._info["hardwareVersion"] == 2:  # If Hardware v2, set correct baud rate (USB 2.0 High Speed)
            self.port.set_baud(480000000)
        self.port.write(ord('N'), 'uint8')  # Request channel and memory info
        self._info["nChannels"] = self.port.read(1, 'uint8').astype(int)[0]  # Read number of channels
        self._info["maxWaveforms"] = self.port.read(1, 'uint16').astype(int)[0]  # Read max number of waveforms
        self._info["maxTriggerProfiles"] = self.port.read(1, 'uint8').astype(int)[0]  # Read max number of trigger profiles

        # Initialize internal variables
        self._waveforms = [[] for _ in range(self._info["maxWaveforms"])]  # Stores all waveforms loaded to the device
        self._waveform_ranges = [[] for _ in range(self._info["maxWaveforms"])]  # Stores ranges of all loaded waveforms

        # Initialize parameters
        self._sampling_rate = None
        self._output_range = None
        self._trigger_mode = None
        self._loop_mode = ReactiveList([False] * self._info["nChannels"], update_callback=self.loop_mode_updater)
        self._loop_duration = ReactiveList([0] * self._info["nChannels"], update_callback=self.loop_duration_updater)
        self._bpod_events = ReactiveList([False] * self._info["nChannels"], update_callback=self.bpod_events_updater)

        # Set default parameters on device by calling set methods
        self.sampling_rate = 10000  # Sampling rate of the device, affecting all channels. Units = Hz
        self.output_range = '-5V:5V'  # Voltage range of 16-bit output. Valid ranges are in self._info["outputRangeList"]
        self.trigger_mode = 'Normal'  # Sets playback start behavior. Valid modes are in self._info["triggerModeList"]
        self.loop_mode = [False] * self._info["nChannels"]  # Loop mode for each channel. True/False = Enabled/Disabled
        self.loop_duration = [0] * self._info["nChannels"]  # Duration of waveform loop on trigger. Units = seconds
        self.bpod_events = [False] * self._info["nChannels"]  # Bpod events for each channel. True/False = Enabled/Disabled

    @property
    def sampling_rate(self):
        return self._sampling_rate

    @sampling_rate.setter
    def sampling_rate(self, sf):
        """ Sets the sampling rate
            Args:
                sf (uint32) The new sampling rate. Units = Hz
            Returns:
                none
        """
        max_sampling_rate = self._info["maxSamplingRate"]
        if sf < 1 or sf > max_sampling_rate:
            raise WavePlayerError(f"Sampling rate must be in range 1, {max_sampling_rate}")
        sampling_period_us = (1 / sf) * 1000000
        self.port.write(ord('S'), 'uint8', sampling_period_us, 'float32')
        self._confirm_transfer('setting sampling rate')
        self._sampling_rate = sf

        # Update loop durations if any are non-zero
        if sum(self.loop_duration) > 0:
            self.port.write(ord('D'), 'uint8', [dur * self.sampling_rate for dur in self.loop_duration], 'uint32')
            self._confirm_transfer('setting sampling rate')

    @property
    def output_range(self):
        return self._output_range

    @output_range.setter
    def output_range(self, new_range_string):
        """ Sets the output range
            Args:
                new_range_string, a string with the name of the range.
                Valid names are in self._info["outputRangeList"]
            Returns:
                none
        """
        # Find the range index
        try:
            range_index = self._info["outputRangeList"].index(new_range_string)
        except ValueError:
            valid_ranges = self._info["outputRangeList"]
            raise WavePlayerError(f"Invalid range: {new_range_string}. Valid ranges are: {valid_ranges}")

        # Check for existing waveforms with voltages out of range
        range_limits = self._info["outputRangeLimits"][range_index]
        for i_wave in range(self._info["maxWaveforms"]):
            if len(self._waveforms[i_wave]) > 0:
                wave_limits = self._waveform_ranges[i_wave]
                if wave_limits[0] < range_limits[0] or wave_limits[1] > range_limits[1]:
                    raise WavePlayerError(f"Error setting range: Previously loaded waveform# {i_wave} " +
                                          "exceeds the requested voltage range.")

        # Update the device
        self.port.write([ord('R'), range_index], 'uint8')
        self._confirm_transfer('setting output range')

        # Update the object
        self._output_range = new_range_string

        # Re-load the waveforms, encoded to bits with the new range
        for i_wave in range(self._info["maxWaveforms"]):
            if len(self._waveforms[i_wave]) > 0:
                self.load(i_wave, self._waveforms[i_wave])

    @property
    def trigger_mode(self):
        return self._trigger_mode

    @trigger_mode.setter
    def trigger_mode(self, new_mode_string):
        """ Sets the trigger mode
            Args:
                new_mode_string, a string with the name of the mode.
                Valid names are in self._info["triggerModeList"]
            Returns:
                none
        """

        # Find the trigger mode index
        try:
            trigger_mode_index = self._info["triggerModeList"].index(new_mode_string)
        except ValueError:
            valid_modes = self._info["triggerModeList"]
            raise WavePlayerError(f"Invalid trigger mode: {new_mode_string}. Valid modes are: {valid_modes}")

        # Update the device
        self.port.write([ord('T'), trigger_mode_index], 'uint8')
        self._confirm_transfer('setting trigger mode')

        # Update the object
        self._trigger_mode = new_mode_string

    @property
    def loop_mode(self):
        return self._loop_mode

    @loop_mode.setter
    def loop_mode(self, new_modes):
        """ Calls loop_mode_updater() whenever loop_mode is assigned a new value, e.g. loop_mode = [True,True,True,True]
            The ReactiveList class calls the update when elements of loop_mode are set, e.g. loop_mode[0] = False
            Args:
                new_modes, a 1 x nChannels list of booleans. True to enable loop mode, False if not
            Returns:
                none
        """
        self._loop_mode = ReactiveList(new_modes, update_callback=self.loop_mode_updater)
        self.loop_mode_updater(new_modes)

    def loop_mode_updater(self, new_modes):
        """ Sets the loop mode
            Args:
                new_modes, a 1 x nChannels list of booleans. True to enable loop mode, False if not
            Returns:
                none
        """
        # Verify length and type
        n_channels = self._info["nChannels"];
        if len(new_modes) != n_channels or not all(isinstance(item, bool) for item in new_modes):
            raise WavePlayerError(f"Error setting loop_mode: loop_mode must be a 1x{n_channels} list of booleans.")

        # Update the device
        int_modes = [int(item) for item in new_modes]  # Convert bool to int
        self.port.write([ord('O')] + int_modes, 'uint8')
        self._confirm_transfer('setting loop mode')

    @property
    def loop_duration(self):
        return self._loop_duration

    @loop_duration.setter
    def loop_duration(self, new_durations):
        """ Calls loop_duration_updater() whenever loop_duration is assigned a new value, e.g. loop_duration = [1,2,3,4]
            The ReactiveList class calls the update when elements of loop_duration are set, e.g. loop_duration[0] = 1
            Args:
                new_durations, a 1 x nChannels list of floats indicating time to loop the triggered waveform (s)
            Returns:
                none
        """
        self._loop_duration = ReactiveList(new_durations, update_callback=self.loop_duration_updater)
        self.loop_duration_updater(new_durations)

    def loop_duration_updater(self, new_durations):
        """ Sets the loop duration
            Args:
                new_durations, a 1 x nChannels list of floats indicating time to loop the triggered waveform (s)
            Returns:
                none
        """
        # Verify length and type
        n_channels = self._info["nChannels"]
        if len(new_durations) != n_channels:
            raise WavePlayerError(f"Error setting loop_duration: loop_duration must be a 1x{n_channels} list of durations (units = seconds).")

        # Update the device
        self.port.write(ord('D'), 'uint8', [dur*self.sampling_rate for dur in new_durations], 'uint32')
        self._confirm_transfer('setting loop durations')

    @property
    def bpod_events(self):
        return self._bpod_events

    @bpod_events.setter
    def bpod_events(self, new_events):
        """ Calls bpod_events_updater() whenever bpod_events is assigned, e.g. bpod_events = [True,True,True,True]
            The ReactiveList class calls the update when elements of bpod_events are set, e.g. bpod_events[0] = False
            Args:
                new_events, a 1 x nChannels list of booleans. True to enable events, False if not
            Returns:
                none
        """
        self._bpod_events = ReactiveList(new_events, update_callback=self.bpod_events_updater)
        self.bpod_events_updater(new_events)

    def bpod_events_updater(self, new_events):
        """ Sets a flag for each channel to return events to the state machine on playback start/stop (or not)
            Args:
                new_events, a 1 x nChannels list of booleans. True to send events, False if not
            Returns:
                none
        """
        # Verify length and type
        n_channels = self._info["nChannels"]
        if len(new_events) != n_channels or not all(isinstance(item, bool) for item in new_events):
            raise WavePlayerError(f"Error setting bpod_events: bpod_events must be a 1x{n_channels} list of booleans.")

        # Update the device
        int_modes = [int(item) for item in new_events]  # Convert bool to int
        self.port.write([ord('V')] + new_events, 'uint8')
        self._confirm_transfer('setting enable/disable bpod events')

    def load(self, wave_index, waveform):
        """ Loads a waveform to a target position in the WavePlayer module's memory.
            Args:
                wave_index, the target position in range 0, 63
                waveform, a 1 x nSamples list of voltages
            Returns:
                none
        """
        max_waveforms = self._info["maxWaveforms"]

        # Ensure valid wave_index
        if wave_index < 0 or wave_index > max_waveforms:
            raise WavePlayerError(f"Invalid waveform index: {wave_index}. Indexes must be in range 0, {max_waveforms}")

        # Convert to bits
        n_samples = len(waveform)
        wave_bits = self._volts_to_bits(waveform)

        # Send to device and confirm transfer
        self.port.write([ord('L'), wave_index], 'uint8', n_samples, 'uint32', wave_bits, 'uint16')
        self._confirm_transfer('loading waveform')

        # Store waveform locally. It will be automatically re-loaded to the device if the range is changed.
        self._waveforms[wave_index] = waveform
        self._waveform_ranges[wave_index] = [min(waveform), max(waveform)]

    def play(self, channels, wave_indexes):
        """ Plays a waveform previously loaded to the WavePlayer module with load()
            Args:
                channels, a list of channels to trigger, in range 1-4. A single channel may be passed as int.
                wave_indexes, a list of indexes of waveforms to play. If one index is provided, that waveform will be
                              played on all channels in channels. If multiple indexes are provided, a separate waveform
                              must be set to begin playback on each channel (omit channels not triggered from channels).
            Returns:
                none
        """

        # Package scalar arguments into lists if scalars were passed in
        if isinstance(channels, int):
            channels = [channels]
        if isinstance(wave_indexes, int):
            wave_indexes = [wave_indexes]

        # Check args
        n_channels = self._info["nChannels"]
        if len(wave_indexes) > 1 and len(channels) != len(wave_indexes):
            raise WavePlayerError(f"Error using play(): One wave index must be specified to play on each channel")
        if min(channels) < 0 or max(channels) > n_channels-1:
            raise WavePlayerError(f"Error using play(): Channels must be in range 0,{n_channels-1}.")

        # Create list of waveforms to trigger on each output channel
        trigger_waves = [255] * n_channels
        for i_chan in range(len(channels)):
            if len(wave_indexes) == 1:
                trigger_waves[channels[i_chan]] = wave_indexes[0]
            else:
                trigger_waves[channels[i_chan]] = wave_indexes[i_chan]

        self.port.write([ord('>')] + trigger_waves, 'uint8')

    def stop(self):
        """  Stops ongoing waveform playback
             Args:
                 none
             Returns:
                 none
         """
        self.port.write(ord('X'), 'uint8')

    def set_fixed_voltage(self, channels, voltage):
        """
        Sets output channel(s) to output a fixed voltage
        Args:
           channels, a list of channels to set
           voltage, the fixed voltage
        Returns:
           None
        """
        if isinstance(channels, int):
            channels = [channels]

        dac_bits = self._volts_to_bits(voltage)
        channelBits = 0
        for channel in channels:  # Convert list of target channels into bits of a byte
            channelBits += 2 ** (channel - 1)
        self.port.write([ord('!'), channelBits], 'uint8', dac_bits, 'uint16')  # Transfer the new fixed voltage
        self._confirm_transfer('setting fixed output voltage')

    def _volts_to_bits(self, volts):
        """
        Convert volts to DAC bits.
        Args:
           volts, a list of voltages to convert
        Returns:
           bits, a list of DAC bits for each voltage in volts, computed for the currently selected range
        """
        range_mapping = {
            '0V:5V': (5, 1),
            '0V:10V': (10, 1),
            '0V:12V': (12, 1),
            '-5V:5V': (10, 0),
            '-10V:10V': (20, 0),
            '-12V:12V': (24, 0)
        }

        if self.output_range not in range_mapping:
            raise WavePlayerError(f"Invalid OutputRange: {self.output_range}")

        voltage_width, positive_only = range_mapping[self.output_range]

        if isinstance(volts, int):
            volts = [volts]

        min_volts = min(volts)
        max_volts = max(volts)
        max_range = (voltage_width / 2) + (positive_only * (voltage_width / 2))
        min_range = -1 * (voltage_width / 2) * (1 - positive_only)

        if min_volts < min_range or max_volts > max_range:
            raise WavePlayerError(
                f"Error converting volts to bits: All voltages must be within the current range: {self.output_range}.")

        offset = (voltage_width / 2) * (1 - positive_only)
        return [ceil(((volt + offset) / voltage_width) * (2 ** 16 - 1)) for volt in volts]

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8')
        if msg != 1:
            raise WavePlayerError("Error " + transfer_description + ". Confirm code not returned.")

    def __repr__(self):
        """  Self-description when the object is entered into the Python shell
             with no properties or methods specified
        """
        return ('BpodWavePlayer with user properties:' + '\n\n'
                'port: ArCom(' + self.port.serialObject.port + ')' + '\n'
                'sampling_rate: ' + str(self.sampling_rate) + '\n'                                                           
                'output_range: ' + self.output_range + '\n'
                'trigger_mode: ' + self.trigger_mode + '\n'
                'loop_mode: ' + str(self.loop_mode) + '\n'
                'loop_duration: ' + str(self.loop_duration) + '\n'
                'bpod_events: ' + str(self.bpod_events) + '\n'
                )

    def __del__(self):
        self.port.close()


class WavePlayerError(Exception):
    pass
