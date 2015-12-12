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
import time
import traceback
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('thrusters_controller')

from seabotix_interface import SeabotixInterface
from vehicle_interface.msg import ThrusterCommand, ThrusterFeedback
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# ROS node parameters
RATE_LOOP = 20              # Hz
INTERVAL_RESET = 0.250      # sec

DEFAULT_LIMIT = 85      # default speeds limits; operating at higher rpms is not recommended from seabotix thrusters
DEFAULT_I2C_RATE = 20   # kHz

# ROS topics
TOPIC_STATUS = 'thrusters/status'
TOPIC_CMDS = 'thrusters/commands'
SRV_SWITCH = 'thrusters/switch'


# CONTROLLER status
STATUS_NORM = 0       # normal operation
STATUS_STOP = 1       # safe-stop operation (if received a safe stop, doesn't trigger safety switch)
STATUS_EMERG = 2      # emergency operation (if received an emergency stop, does trigger safety switch)
STATUS_ERROR = 4      # safe-stop operation (if lost control of thrusters)

THRESH_ERR = 5      # maximum allowed errors per thrusters
THRESH_TIME = 20    # time between error counter resets (seconds)


# NESSIE configuration (maybe load from file?)
THRUSTERS_SPEC = [
    {'name': 'fwd-port',   'addr': 0x52, 'mux': 0x70, 'chan': 0, 'limits': 75, 'axis': 'x', 'dir': 'normal', 'x_offset': 0, 'y_offset': -0.175, 'z_offset': 0},
    {'name': 'fwd-stbd',   'addr': 0x52, 'mux': 0x70, 'chan': 3, 'limits': 75, 'axis': 'x', 'dir': 'normal', 'x_offset': 0, 'y_offset': -0.175, 'z_offset': 0},
    {'name': 'lat-rear',   'addr': 0x52, 'mux': 0x70, 'chan': 2, 'limits': 75, 'axis': 'y', 'dir': 'normal', 'x_offset': 0, 'y_offset': -0.175, 'z_offset': 0},
    {'name': 'lat-front',  'addr': 0x52, 'mux': 0x71, 'chan': 0, 'limits': 75, 'axis': 'y', 'dir': 'normal', 'x_offset': 0, 'y_offset': -0.175, 'z_offset': 0},
    {'name': 'vert-rear',  'addr': 0x52, 'mux': 0x70, 'chan': 1, 'limits': 75, 'axis': 'z', 'dir': 'normal', 'x_offset': 0, 'y_offset': -0.175, 'z_offset': 0},
    {'name': 'vert-front', 'addr': 0x52, 'mux': 0x71, 'chan': 1, 'limits': 75, 'axis': 'z', 'dir': 'normal', 'x_offset': 0, 'y_offset': -0.175, 'z_offset': 0},
]


class ThrustersDriver(object):

    def __init__(self, name, i2c_rate, i2c_pull, i2c_power, limit, trigger=True, thresh_err=THRESH_ERR, thresh_time=THRESH_TIME):
        self.name = name

        # configuration
        self.config = THRUSTERS_SPEC
        self.throttles = np.zeros((len(self.config), ))         # init speed vectors using spec
        self.limits = limit * np.ones_like(self.throttles)      # init limits using default conf

        # low-level interface
        self.interface = SeabotixInterface(self.config, i2c_rate, i2c_pull, i2c_power)
        self.interface.reset()

        # enable protection mode if thrusters errors?
        self.trigger_protection = trigger
        self.thresh_err = thresh_err
        self.thresh_time = thresh_time

        # controller status and timing
        self.ctrl_status = STATUS_NORM    # internal fsm status
        self.t_loop = time.time()       # current loop time
        self.t_reset = time.time()      # time of last error reset

        # ros interface
        self.pub_stat = rospy.Publisher(TOPIC_STATUS, ThrusterFeedback, tcp_nodelay=True, queue_size=1)
        self.sub_cmds = rospy.Subscriber(TOPIC_CMDS, ThrusterCommand, self._handle_commands, queue_size=1, tcp_nodelay=True)
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

        # timers
        self.t_cmds = rospy.Timer(rospy.Duration(INTERVAL_RESET), self._handle_timer_cmds)      # commands safety timer



    # TIMERs related
    def _clear_timer_cmds(self):
        """Resets the timer used for forcing requested speeds to zero"""
        self.t_cmds.shutdown()
        self.t_cmds = rospy.Timer(rospy.Duration(INTERVAL_RESET), self._handle_timer_cmds)

    def _handle_timer_cmds(self, event):
        """Resets required speeds to zero if timer expires"""
        self.throttles = np.zeros_like(self.throttles)
        rospy.logdebug('%s resetting thrusters speeds ...', self.name)


    # SERVICEs related
    def handle_switch(self, req):
        if req.request is True:
            self.ctrl_status = STATUS_NORM
            return BooleanServiceResponse(True)
        else:
            self.ctrl_status = STATUS_STOP
            return BooleanServiceResponse(False)


    # SUBSCRIBERs related
    def _handle_commands(self, data):
        """Processes input commands from high-level modules"""
        try:
            # avoid out-of-boundary errors
            throttle = np.array(data.throttle[0:6])
        except Exception:
            rospy.logerr('%s bad input command, skipping!')
        else:
            # calculate thruster limits and accept commands
            self.throttles = np.clip(throttle, -self.limits, self.limits)

            # reset commands timer
            self._clear_timer_cmds()


    # LOGIC related
    def loop(self):
        self.t_loop = time.time()

        # fsm state
        if self.ctrl_status == STATUS_NORM:
            # normal mode and reset counters at a slower rate (prevents spurious errors to trigger the protection)
            if self.t_loop - self.t_reset > self.thresh_time:
                self.interface.errors = np.zeros_like(self.interface.errors)    # reset error counters
                self.t_reset = self.t_loop
        else:
            # any stopped mode (keep sending zeros to thrusters)
            #   note: this state needs an external reset for safety reasons!
            self.throttles = np.zeros_like(self.throttles)      # set speed to zero

        # check interface errors and prevent overflows
        if np.any(self.interface.errors > self.thresh_err):
            self.interface.errors = np.zeros_like(self.interface.errors)    # reset error counters

            # abort operations if protection is enabled (state change)
            if self.trigger_protection:
                rospy.logerr('%s error threshold reached -> protection mode activated!', self.name)
                self.ctrl_status = STATUS_ERROR


        try:
            # send thrusters commands
            self.interface.process_thrusters(self.throttles)
        except Exception as e:
            rospy.logerr('%s thrusters interface error:\n%s', name, e)
            traceback.print_exc(e)

            self.interface.shutdown()   # try a clean shutdown of thrusters
            raise e                     # and rethrow the exception

        # publish report
        tf = ThrusterFeedback()
        tf.header.stamp = rospy.Time.now()
        tf.throttle = self.interface.feedback_throttle.tolist()
        tf.current = self.interface.feedback_curr.tolist()
        tf.temp = self.interface.feedback_temp.tolist()
        tf.status = self.interface.feedback_fault.tolist()
        tf.errors = self.interface.errors.tolist()
        self.pub_stat.publish(tf)



    def shutdown(self):
        """safe shutdown and safety hooks"""
        rospy.loginfo('%s shutdown procedure', self.name)
        self.sub_cmds.unregister()
        self.t_cmds.shutdown()

        self.interface.shutdown()
        rospy.loginfo('%s shutdown completed!', self.name)



if __name__ == '__main__':
    rospy.init_node('thrusters_driver')
    name = rospy.get_name()

    # aardvark parameters
    i2c_rate = int(rospy.get_param('~i2c_rate', DEFAULT_I2C_RATE))
    i2c_pull = bool(rospy.get_param('~i2c_pullup', False))
    i2c_power = bool(rospy.get_param('~i2c_power', False))

    # thrusters parameters
    lim = int(rospy.get_param('thrusters/throttle_limit', DEFAULT_LIMIT))
    trigger = bool(rospy.get_param('~protection', True))
    thresh_err = int(rospy.get_param('~errors_threshold', THRESH_ERR))
    thresh_time = int(rospy.get_param('~errors_time', THRESH_TIME))

    # apply limits
    lim = int(np.clip(lim, 0, 100))
    thresh_err = int(np.clip(thresh_err, 0, 100))
    thresh_time = int(np.clip(thresh_time, 0, 1000))


    # log configuration
    rospy.loginfo('%s initializing ... ', name)
    rospy.loginfo('%s throttle limit set to: %d%%', name, lim)
    rospy.loginfo('%s protection trigger: %s', name, trigger)
    rospy.loginfo('%s maximum errors: %s', name, thresh_err)
    rospy.loginfo('%s errors reset every: %s secs', name, thresh_time)

    rospy.loginfo('%s i2c rate: %s', name, i2c_rate)
    rospy.loginfo('%s i2c pullup: %s', name, i2c_pull)
    rospy.loginfo('%s i2c power: %s', name, i2c_power)


    # run the controller
    tc = ThrustersDriver(name, i2c_rate, i2c_pull, i2c_power, limit=lim, trigger=trigger, thresh_err=thresh_err, thresh_time=thresh_time)
    rate_loop = rospy.Rate(RATE_LOOP)   # main loop rate Hz

    while not rospy.is_shutdown():
        try:
            tc.loop()
            rate_loop.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
            tc.shutdown()
        except Exception as e:
            rospy.logfatal('%s caught exception and dying!', name)
            tc.shutdown()

            traceback.print_exc(e)
            sys.exit(-1)
