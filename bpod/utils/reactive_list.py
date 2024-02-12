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

# This class exposes the __setitem__ event of a list, to trigger a user callback when list elements are set.
# The callback is used in Bpod to update the device when a parameter is exposed as a list with a value for each channel.
# e.g. this line of code would update my_parameter of channel 2 on MyDevice:
# MyDevice.my_parameter[2] = new_value


class ReactiveList(list):

    def __init__(self, *args, update_callback=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.update_callback = update_callback

    def __setitem__(self, key, value):
        super().__setitem__(key, value)
        if self.update_callback:
            self.update_callback(self)  # pass the entire list
