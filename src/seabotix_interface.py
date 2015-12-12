#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Heriot-Watt University nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Original authors:
#   Valerio De Carolis, Marian Andrecki, Corina Barbalata, Gordon Frost

from __future__ import division

import sys
import traceback
import numpy as np
np.set_printoptions(precision=3, suppress=True)

# aardvark lib import (32bit and 64bit)
# detect bit-width of current platform and import aadvark.so
if sys.maxsize > 2**32:
    from aardvark_64.aardvark_py import *
else:
    from aardvark_32.aardvark_py import *


# Thrusters Interface Constants
DEFAULT_LIMIT = 85      # default speeds limits; operating at higher speeds will deteriorate the thrusters (max 6A on datasheet)
DEFAULT_I2C_RATE = 20   # kHz

# MUX I2C interface
MUX_FWD = 0x70          # I2C 7bit address
MUX_AFT = 0x71          # I2C 7bit address
MUX_CTRL = 0x04         # MUX control register
MUX_RESET = 0x00        # MUX reset byte

# Seabotix I2C addresses
SBX_ADDR        = 0x52
SBX_ADDR_I2C    = SBX_ADDR >> 1

# Seabotix thrusters possible speed values:
#   Forwards:  129 (0x81) (min) to 230 (0xE6) (max)
#   Zero:      127 (0x7F) or 128 (0x80)
#   Reverse:   126 (0x7E) (min) to 25 (0x19) (max)
#
# So in each direction, the range is 102 values, excluding double zeros.
# For sake of simplicity only 100 values can considered:
#    Forwards: 129 to 228
#    Reverse: 126 to 27
#
# Or directly with a limited contiguous range like implemented below:
#   -100 to 100 is mapped to 28 to 228
#   in this way -1% reverse will be treated as 0 command
#   but the calculation is straightforward just apply an offset
#
# Moreover MIN and MAX boundaries are enforced in this code.
SBX_ZERO    = 128   #0x80  # 128
SBX_MAX     = 228   #0xE4  # 228
SBX_MIN     = 28    #0x1C  # 28
SBX_OFFSET  = 128   # plain offset

# required parameters
REQ_PARAMS = ('addr', 'mux', 'chan')


class SeabotixInterface(object):
    """This class provides a python interface for Seabotix thrusters on Nessie AUV.

        The thrusters' interface is using a TotalPhase Aardvark and I2C muxes to communicate with individual devices.

        Take extra care in handling I2C address as the ones written in the Seabotix's manual includes an 8th bit for
        differentiating between read and write operations. The I2C bus itself is using 7bit address.

        Please refer to officials data sheets and manuals before modify this code.
    """

    def __init__(self, config, i2c_rate=DEFAULT_I2C_RATE, i2c_pull=False, i2c_power=False):
        self.thrs_spec = config
        self.thrs_num = len(config)

        # check thruster config
        if self.thrs_num != 6:
            raise ValueError('SBX: thrusters spec is not of right length!')

        for thruster in self.thrs_spec:
            if not all(param in thruster for param in REQ_PARAMS):
                print(thruster)
                raise ValueError('SBX: thrusters spec is not valid!')


        self.errors = np.zeros(self.thrs_num)

        self.feedback_throttle = np.zeros(self.thrs_num)
        self.feedback_curr = np.zeros(self.thrs_num)
        self.feedback_temp = np.zeros(self.thrs_num)
        self.feedback_fault = np.zeros(self.thrs_num)

        # initialize the i2c interface
        self.handle = self.__init_aardvark(i2c_rate, i2c_pull, i2c_power)



    def __init_aardvark(self, i2c_rate, i2c_pull, i2c_power):
        # find all the attached devices
        (num, ports, unique_ids) = aa_find_devices_ext(16,16)

        if num == 0:
            print('SBX: no Aardvark device found!')
            sys.exit(1)

        # Open Aardvark device
        handle = aa_open(ports[0])

        if handle <= 0:
            print('SBX: error opening Aardvark device (%d)' % handle)
            sys.exit(1)
        else:
            print('SBX: opened Aardvark device (%d)' % handle)

        # Ensure that the I2C subsystem is enabled
        aa_configure(handle,  AA_CONFIG_SPI_I2C)

        # Disable the I2C bus pullup resistors (2.2k resistors).
        # This command is only effective on v2.0 hardware or greater.
        # The pullup resistors on the v1.02 hardware are enabled by default.
        if i2c_pull is True:
            aa_i2c_pullup(handle, AA_I2C_PULLUP_BOTH)
            print('SBX: Aardvark I2C pullups enabled')
        else:
            aa_i2c_pullup(handle, AA_I2C_PULLUP_NONE)
            print('SBX: Aardvark I2C pullups disabled')

        # Disable the Aardvark adapter's power pins.
        # This command is only effective on v2.0 hardware or greater.
        # The power pins on the v1.02 hardware are not enabled by default.
        if i2c_power is True:
            aa_target_power(handle, AA_TARGET_POWER_BOTH)
            print('SBX: Aardvark I2C power enabled')
        else:
            aa_target_power(handle, AA_TARGET_POWER_NONE)
            print('SBX: Aardvark I2C power disabled')

        # Set the bitrate
        i2c_rate_eff = aa_i2c_bitrate(handle, i2c_rate)
        print('SBX: I2C bitrate %d kHz' % i2c_rate_eff)

        return handle



    # MUX functions
    def _mux_select(self, addr, channel):
        """Select a multiplexer channel by writing to its control address
            @returns:   (count, data_in)
        """
        data_out = array('B', [MUX_CTRL | channel])
        return aa_i2c_write(self.handle, addr, AA_I2C_NO_FLAGS, data_out)

    def _mux_reset(self, addr):
        """Resets the multiplexer by writing to its control address
            @returns:   (count, data_in)
        """
        data_out = array('B', [MUX_RESET & 0xFF])
        return aa_i2c_write(self.handle, addr, AA_I2C_NO_FLAGS, data_out)

    def _mux_status(self, addr):
        """Returns multiplexer status by reading its control address
            @returns:   (count, data_in)
        """
        return aa_i2c_read(self.handle, addr, AA_I2C_NO_FLAGS, 1)


    # Seabotix functions
    def _sbx_status(self, addr):
        """Read status from Seabotix thruster using I2C bus.

            This code reads the "Seabotix Motor Controller Thruster Motor Controller To LBV
            Motherboard Message" - see Seabotix "Interface Design Specification Between
            LBV Motherboard and Thruster Motor Controller". Here's a summary:
            Byte 0: Address of thruster
            Byte 1: Status, bits:
                  0: 1 = non-current limiting version of firmware, 0 = current limiting
                  1: 1 = brushed motor, 0 = brushless
                  2: future growth
                  3: future growth
                  4-7: revision number of firmware
            Byte 2: Fault, bits:
                  0: 1 = overtemp
                  1: 1 = stalled motor
                  2: 2 = hall sensor error
                  3: 3 = ground fault
                  4: 4 = water detect
                  5-6: future growth
            Byte 3: Current: for brushless DC software, indicates drawn current in 1/10 Amps.
            Byte 4: Speed: actual speed of the thruster (see document for definition).
            Byte 5: Temperature: motor temp in deg C, or motor controller temp for brushed motors.
            Byte 6: Checksum: eight bit unsigned addition without carry of above fields.

            Returns a tuple of values:
                (count, speed, current, temp, fault, info)
        """
        (count, data_in) = aa_i2c_read(self.handle, addr, AA_I2C_NO_FLAGS, 7)

        # check output
        if count != 7:
            return (-count, 0, 0, 0, 0, 0)

        # checksum check
        csum = sum(data_in[0:-1])   # calculate the checksum using first 5 bytes

        if csum != data_in[6]:
            #print('csum: %s, data: %s', csum, data_in[6])
            #count = -count  # signal error and continues
            pass

        return (count, data_in[4], data_in[3], data_in[5], data_in[2], data_in[1])


    def _sbx_command(self, addr, speed):
        """Send command to Seabotix thruster"""

        data_out = array('B', [0, 0, 0])
        data_out[0] = speed     # requested speed
        data_out[1] = 100       # fixed value (see manual)

        # checksum generation
        data_out[2] = ((SBX_ADDR + data_out[0] + data_out[1]) % 256)

        # send command
        return aa_i2c_write(self.handle, addr, AA_I2C_NO_FLAGS, data_out)



    # INTERFACE function
    def process_thrusters(self, throttle_request):
        """Send commands to thrusters and update the feedback variables.

            This function send commands to all thrusters specified in the interface configuration.
        """

        # calculate the raw thruster commands
        throttle = np.copy(throttle_request)                # local copy of speeds (callback protection)
        throttle = throttle + SBX_OFFSET                    # apply offset
        throttle = np.clip(throttle, SBX_MIN, SBX_MAX)      # limit commands

        # thrusters loop
        for th_idx, th_req in enumerate(throttle):
            # set mux and select thruster
            addr = int(self.thrs_spec[th_idx]['addr'] >> 1)     # shift to get 7bit I2C address
            mux = int(self.thrs_spec[th_idx]['mux'])            # from config
            chan = int(self.thrs_spec[th_idx]['chan'])          # from config
            spd = int(th_req)


            # mux selection
            res = self._mux_select(mux, chan)

            if res == 0:
                self.errors[th_idx] += 1
                print('SBX: mux error: id: %s, addr: %s, mux: %s, chan: %s' % (th_idx, hex(addr), hex(mux), chan))

            # send command to thruster
            res = self._sbx_command(addr, spd)

            if res == 0:
                self.errors[th_idx] += 1
                #print('SBX: write error: id: %s, addr: %s, mux: %s, chan: %s' % (th_idx, hex(addr), hex(mux), chan))
            # else:
            #    aa_sleep_ms(50)

            # query thruster
            (count, speed, current, temp, fault, info) = self._sbx_status(addr)

            if count != 7:
                self.errors[th_idx] += 1
                #print('SBX: read error: id: %s, addr: %s, mux: %s, chan: %s, count: %s' % (th_idx, hex(addr), hex(mux), chan, count))

            # process raw values and produce nice data
            #   throttle:     normalize between -100 and 100
            #   currents:   divide by ten to get amps (otherwise output is tenth of amps)
            self.feedback_throttle[th_idx] = np.clip(speed - SBX_OFFSET, -100, 100)
            self.feedback_curr[th_idx] = np.clip(current / 10.0, 0, 100)
            self.feedback_temp[th_idx] = temp
            self.feedback_fault[th_idx] = fault

            # reset mux
            res = self._mux_reset(mux)

            if res == 0:
                self.errors[th_idx] += 1
                print('SBX: mux error: id: %s, addr: %s, mux: %s, chan: %s' % (th_idx, hex(addr), hex(mux), chan))

            # query mux status
            #(count, data_in) = self._mux_status(MUX_FWD)
            #print('SBX: mux selected status: cnt: %s reg: %s' % (count, data_in[0]))


    def reset(self):
        """Reset the interface.

            This sends a reset command to all multiplexers preparing the bus if not sure about its state.
        """
        self._mux_reset(MUX_FWD)
        self._mux_reset(MUX_AFT)


    def shutdown(self):
        """Close safely the interface.

            This resets the interface multiplexers and sends zero throttle to all thrusters before closing the
            Aardvark device. This is considered to be a safe shutdown to be invoked if any logical exception is caught.
        """

        # initial mux reset
        self._mux_reset(MUX_FWD)
        self._mux_reset(MUX_AFT)

        # send zero commands before closing
        for n in xrange(5):
            self.process_thrusters(np.zeros(self.thrs_num))     # send commands
            aa_sleep_ms(100)                                    # wait for aardvark

        # final mux reset
        self._mux_reset(MUX_FWD)
        self._mux_reset(MUX_AFT)

        # disable and close the device
        print('SBX: closing Aardvark device!')
        aa_close(self.handle)


if __name__ == '__main__':
    print('Seabotix Interface: starting test mode')

    try:
        sb = SeabotixInterface(config=None)
        sb.shutdown()
    except Exception as e:
        traceback.print_exc(e)
        sys.exit(-1)
    else:
        sys.exit(0)
