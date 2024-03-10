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

# BpodHiFi is a class to control the Bpod HiFi Module via its USB connection to the PC.
# The HiFi module stores up to 20 stereo 192kHz waveforms, and plays them back on trigger.
#
# ---Usage example---
# #Imports
# from bpod.modules.hifi import BpodHiFi
# import numpy as np
# import time
#
# #Sound parameters:
# sf = 192000 # Sampling rate
# freq = 1000 # Tone frequency
# duration = 1 # Full tone duration (interrupted by stop command below)
#
# #Setup device
# H = BpodHiFi('COM3') # Replace 'COM3' with the HiFi module's USB serial port
# H.samplingRate = sf
# H.digitalAttenuation_dB = -10 # Digital attenuation. For best fidelity, set to 0 and attenuate with your speaker amp
#
# #Create and load waveform
# wave = np.sin(2 * np.pi * freq * np.arange(duration * sf) / sf)
# H.load(0, wave) # Load the waveform to the HiFi module's memory at position 0
# H.push()  # Adds newly loaded sounds to the active sound set
# envelope = np.linspace(0,1,1920) # 10ms linear ramp (1920 samples at 192kHz sampling)
# H.set_am_envelope(envelope)  # The envelope will be applied to the waveform on sound start, and in reverse on stop.
#                              Each sample is an attenuation factor ranging between 1 (full amplitude) and 0 (no signal)
# H.play(0)  # Play waveform 0
# time.sleep(0.5)
# H.stop()  # Stop playback
# del H  # Clear the BpodHiFi object

from bpod.utils.arcom import ArCom
import numpy as np


class BpodHiFi(object):
    def __init__(self, port_name):
        """  Initializes the BpodHiFi object, opens the USB serial port and configures the device
             Args:
                 port_name (string) The name of the USB serial port as known to the OS
                                    Examples = 'COM3' (Windows), '/dev/ttyACM0' (Linux)
        """

        # Connect to device
        self.port = ArCom(port_name, 480000000)

        # Get current parameters
        self.port.write(ord('I'), 'uint8')
        info_params8_bit = self.port.read(4, 'uint8')
        info_params32_bit = self.port.read(3, 'uint32')

        # Set internal parameters  (synchronizes defaults to the device)
        self._is_hd = info_params8_bit[0]  # HiFi Module Model (0=base,1=HD)
        self._sampling_rate = info_params32_bit[0]  # Units = Hz
        self._bit_depth = info_params8_bit[1]  # Bits per sample (fixed in firmware)
        self._max_waves = info_params8_bit[2]  # Number of sounds the device can store
        self._max_samples_per_waveform = info_params32_bit[1]*192000
        self._max_envelope_samples = info_params32_bit[2]
        self._valid_sampling_rates = (44100, 48000, 96000, 192000)
        self._min_attenuation_pro = -103  # Minimum digital attenuation on SD model (units = dB FS)
        self._min_attenuation_hd = -120  # Minimum digital attenuation on HD model (units = dB FS)

        # Set default user parameters
        self.sampling_rate = 192000  # Sampling rate for audio playback. Affects all sounds. Units = Hz
        self.digital_attenuation_db = -1  # Digital attenuation. 0 = full-scale output. Min attenuation is per model (SD or HD)
        self.synth_amplitude = 0  # 0 = synth off, 1 = max
        self.synth_amplitude_fade = 0  # Number of samples to ramp changes in synth amplitude (0 = instant change)
        self.synth_waveform = 'WhiteNoise'  # 'WhiteNoise' or 'Sine'
        self.synth_frequency = 1000  # Frequency of waveform (if sine)
        self.headphone_amp_enabled = 0  # Set to 1 to enable headphone amp
        self.headphone_amp_gain = 15  # Headphone amp gain. Range = 0-63

    @property
    def sampling_rate(self):
        return self._sampling_rate

    @sampling_rate.setter
    def sampling_rate(self, value):
        """ Sets the sampling rate globally on the HiFi module
            Args:
                value (uint32) The new sampling rate. Units = Hz
            Returns:
                none
        """
        if value not in self._valid_sampling_rates:
            raise HiFiError('Error: Invalid Sampling Rate.')
        self.port.write(ord('S'), 'uint8', value, 'uint32')
        self._confirm_transfer('setting sampling rate')
        self._sampling_rate = value

    @property
    def digital_attenuation_db(self):
        return self._digital_attenuation_db

    @digital_attenuation_db.setter
    def digital_attenuation_db(self, value):
        """ Sets digital attenuation globally on the HiFi module
            Args:
                value (int32) The new attenuation. Units = dB FS
            Returns:
                none
        """
        min_attenuation = self._min_attenuation_pro
        if self._is_hd:
            min_attenuation = self._min_attenuation_hd
        if (value > 0) or (value < min_attenuation):
            raise HiFiError('Error: Invalid digital_attenuation_db. ' +
                            'Value must be in range: [' + str(min_attenuation) + ',0]')
        attenuation_bits = value*-2
        self.port.write((ord('A'),attenuation_bits), 'uint8')
        self._confirm_transfer('setting digital attenuation')
        self._digital_attenuation_db = value

    @property
    def synth_amplitude(self):
        return self._synth_amplitude

    @synth_amplitude.setter
    def synth_amplitude(self, value):
        """ Sets synth amplitude
            Args:
                value (float) The new amplitude. Units = Fractional in range [0, 1]
            Returns:
                none
        """
        if not ((value >= 0) and (value <= 1)):
            raise HiFiError('Error: Synth amplitude values must range between 0 and 1.')
        amplitude_bits = round(value*65535)
        self.port.write(ord('N'), 'uint8', amplitude_bits, 'uint16')
        self._confirm_transfer('setting synth amplitude')
        self._synth_amplitude = value

    @property
    def synth_amplitude_fade(self):
        return self._synth_amplitude_fade

    @synth_amplitude_fade.setter
    def synth_amplitude_fade(self, value):
        """ Sets duration of synth amplitude transitions.
            Amplitude fades to the new level across a configurable number of samples
            Args:
                value (uint32) The new duration. Units = samples
            Returns:
                none
        """
        if not ((value >= 0) and (value <= 1920000)):
            raise HiFiError('Error: Synth amplitude fade must range between 0 and 1920000 samples.')
        self.port.write(ord('Z'), 'uint8', value, 'uint32')
        self._confirm_transfer('setting synth amplitude fade')
        self._synth_amplitude_fade = value

    @property
    def synth_waveform(self):
        return self._synth_waveform

    @synth_waveform.setter
    def synth_waveform(self, value):
        """ Sets the active synth waveform
            Args:
                value (string) The name of the waveform
            Returns:
                none
        """
        valid_waveforms = ['WhiteNoise', 'Sine']
        if value not in valid_waveforms:
            raise HiFiError('Invalid value for synth_waveform: Valid values are WhiteNoise or Sine.')
        self.port.write((ord('W'), valid_waveforms.index(value)), 'uint8')
        self._confirm_transfer('setting synth_waveform')
        self._synth_waveform = value

    @property
    def synth_frequency(self):
        return self._synth_frequency

    @synth_frequency.setter
    def synth_frequency(self, value):
        """ Sets the synth frequency for periodic synth waveforms.
            This setting has no effect if white noise is selected.
            Args:
                value (uint32) The new synth waveform frequency
            Returns:
                none
        """
        if not ((value >= 20) and (value <= 80000)):
            raise HiFiError('Error: synth_frequency must range between 0 and 80000 Hz.')
        self.port.write(ord('F'), 'uint8', value*1000, 'uint32')
        self._confirm_transfer('setting synth_frequency')
        self._synth_frequency = value

    @property
    def headphone_amp_enabled(self):
        return self._headphone_amp_enabled

    @headphone_amp_enabled.setter
    def headphone_amp_enabled(self, value):
        """ Sets the state of the headphone amplifier on the HiFi SD model.
            This setting has no effect if using HiFi HD.
            Args:
                value (uint8) Use 1 for enabled, 0 for disabled
            Returns:
                none
        """
        valid_values = (0, 1)
        if value not in valid_values:
            raise HiFiError('Error: headphone_amp_enabled must be 0 (disabled) or 1 (enabled)')
        self.port.write((ord('H'), value), 'uint8')
        self._confirm_transfer('setting headphone_amp_enabled')
        self._headphone_amp_enabled = value

    @property
    def headphone_amp_gain(self):
        return self._headphone_amp_gain

    @headphone_amp_gain.setter
    def headphone_amp_gain(self, value):
        """ Sets the gain of the headphone amplifier on the HiFi SD model.
             This setting has no effect if using HiFi HD.
             Args:
                 value (uint8) The headphone amp gain increment, in range [0, 63]
             Returns:
                 none
         """
        if not ((value >= 0) and (value <= 63)):
            raise HiFiError('Error: headphone_amp_gain values must range between 0 and 63.')
        self.port.write((ord('G'), value), 'uint8')
        self._confirm_transfer('setting headphone_amp_gain')
        self._headphone_amp_gain = value

    def play(self, sound_index):
        """  Begins playing a previously loaded sound
             Args:
                 sound_index (uint8) The index of the sound to play
             Returns:
                 none
         """
        self.port.write((ord('P'), sound_index), 'uint8')

    def stop(self):
        """  Stops all sound playback
             Args:
                 none
             Returns:
                 none
         """
        self.port.write(ord('X'), 'uint8')

    def push(self):
        """  Adds all recently loaded sounds to the active sound set
             Args:
                 none
             Returns:
                 none
         """
        self.port.write(ord('*'), 'uint8')
        self._confirm_transfer('activating loaded sounds with push()')

    def set_am_envelope(self, envelope):
        """  Loads and activates an attenuation envelope to apply to the sound on playback start,
             and in reverse on playback stop.
             Args:
                 envelope (float) A 1xN array specifying attenuation for each sample in range [0, 1]
             Returns:
                 none
         """
        if not envelope.ndim == 1:
            raise HiFiError('Error: AM Envelope must be a 1xN array.')
        n_envelope_samples = envelope.shape[0]
        if n_envelope_samples > self._max_envelope_samples:
            raise HiFiError('Error: AM Envelope cannot contain more than ' + str(self._max_envelope_samples) + ' samples.')
        if not ((envelope >= 0).all() and (envelope <= 1).all()):
            raise HiFiError('Error: AM Envelope values must range between 0 and 1.')
        self.port.write((ord('E'), 1, ord('M')), 'uint8', n_envelope_samples, 'uint16', envelope, 'float32')
        self._confirm_transfer('enabling AM envelope')  # Confirm envelope enabled
        self._confirm_transfer('setting AM envelope')  # Confirm envelope received

    def load(self, sound_index, waveform):
        """  Loads a new sound waveform to a specific slot in the HiFi module's memory
             Args:
                 sound_index (uint8) The index of the target sound slot
                 waveform (float) A 1xN or 2xN audio waveform. Each sample in the array is in range [-1,1]
                                  If 1XN is provided, the waveform is played on L+R audio channels.
                                  If 2XN is provided, row 1 = left channel, row 2 = right channel
             Returns:
                 none
         """
        is_stereo = 1  # Default to stereo
        is_looping = 0  # Default loop mode = off
        loop_duration = 0  # Default loop duration = 0
        if (sound_index < 0) or (sound_index > self._max_waves - 1):
            raise HiFiError('Error: Invalid sound index (' + str(sound_index) + '). ' +
                            'The HiFi module supports up to ' + str(self._max_waves) + ' sounds.')

        if waveform.ndim == 1:
            is_stereo = 0
            n_samples = waveform.shape[0]
            formatted_waveform = waveform
        else:
            (nChannels, n_samples) = waveform.shape
            if nChannels == 1:
                is_stereo = 0
                formatted_waveform = waveform
            elif nChannels == 2:
                formatted_waveform = np.ravel(waveform, order='F')
            else:
                raise HiFiError('Error: waveforms must be given as lists of 1 x nSamples or 2 x nSamples')
        if self._bit_depth == 16:
            formatted_waveform = formatted_waveform*32767
        self.port.write((ord('L'), sound_index, is_stereo, is_looping), 'uint8',
                        (loop_duration, n_samples), 'uint32', formatted_waveform, 'int16')
        self._confirm_transfer('loading sound to HiFi module')

    def _confirm_transfer(self, transfer_description):
        msg = self.port.read(1, 'uint8')
        if msg != 1:
            raise HiFiError("Error " + transfer_description + ". Confirm code not returned.")

    def __repr__(self):
        """  Self-description when the object is entered into the Python shell
             with no properties or methods specified
        """
        return ('\nBpodHiFi with user properties:' + '\n\n'
                'port: ArCOMObject(' + self.port.serialObject.port + ')' + '\n'
                'sampling_rate: ' + str(self.sampling_rate) + '\n'
                'digital_attenuation_db: ' + str(self.digital_attenuation_db) + '\n'
                'synth_amplitude: ' + str(self.synth_amplitude) + '\n'
                'synth_amplitude_fade: ' + str(self.synth_amplitude_fade) + '\n'
                'synth_waveform: ' + str(self.synth_waveform) + '\n'
                'synth_frequency: ' + str(self.synth_frequency) + '\n'
                'headphone_amp_enabled: ' + str(self.headphone_amp_enabled) + '\n'
                'headphone_amp_gain: ' + str(self.headphone_amp_gain) + '\n'
                )

    def __del__(self):
        self.port.close()


class HiFiError(Exception):
    pass
