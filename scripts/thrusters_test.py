#!/usr/bin/env python

# Generates a speed message for a chosen thruster, sweeps from 0 to 85% to -85% to 0. thruster_id, velocity step,
# time step can be specified as arguments or left as default values. If velocity step of 30 is
# chosen the generated velocities will be 0, 30, 60, 90, 60, 30, 0, -30, etc. (i.e. 100 will not be reached).
# Sine wave signal can be chosen as well. The user needs to specify the thruster, frequency (max 0.5Hz), time step.

# imports
import sys
import numpy as np

import rospy
import roslib
roslib.load_manifest('thrusters_controller')

# msgs and services

from vehicle_interface.msg import ThrusterCommand

# constants
FREQ = 10   # 10Hz
AMPLITUDE = 75
DEFAULT_dt = 5
DEFAULT_dv = 2
DEFAULT_id = 0
CYCLES_LIMIT = 2


class ThrusterTester(object):
    def __init__(self, argv):
        try:
            if len(argv) > 4: self.mode = argv[4]
            else: self.mode = 'linear'
            if len(argv) > 3: self.dt = float(argv[3])
            else: self.dt = DEFAULT_dt
            if len(argv) > 2: self.dv = float(argv[2])
            else: self.dv = DEFAULT_dv
            if len(argv) > 1: self.thruster_id = int(argv[1])
            else: self.thruster_id = DEFAULT_id
        except ValueError:
            rospy.loginfo('Non-numerical value given. Exiting.')
            exit()

        self.msg = ThrusterCommand()
        self.msg.speed = np.round(np.zeros(6))
        self.current_speed = 0
        self.direction = 1
        self.t = 0

        # loop rate
        self.r_loop = rospy.Rate(FREQ)

        # timer
        self.update_speed = rospy.Timer(rospy.Duration(self.dt), self.inc_speed)

        # publisher
        self.pub_cmd = rospy.Publisher('thrusters/commands', ThrusterCommand, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():

            self.msg.header.stamp = rospy.Time.now()
            self.pub_cmd.publish(self.msg)

            self.r_loop.sleep()

    def inc_speed(self, event):
        if self.mode == 'linear':
            if abs(self.current_speed) + self.dv > AMPLITUDE:
                self.direction *= -1
            self.current_speed += self.direction * self.dv
            self.msg.speed[self.thruster_id] = np.round(self.current_speed)

        elif self.mode == 'multiple':
            if abs(self.current_speed) + self.dv > AMPLITUDE:
                self.direction *= -1
            self.current_speed += self.direction * self.dv
            self.msg.speed[2 * self.thruster_id] = np.round(self.current_speed)
            self.msg.speed[2 * self.thruster_id + 1] = np.round(self.current_speed)

        elif self.mode == 'sine':
            # limit the freq to 0.5 max
            self.dv = np.minimum(self.dv, 2)
            self.msg.speed[2 * self.thruster_id] = np.round(AMPLITUDE*np.sin(self.dv * 2*np.pi * self.t)) + 40
            self.msg.speed[2 * self.thruster_id+1] = np.round(AMPLITUDE*np.sin(self.dv * 2*np.pi * self.t)) + 40
            self.t += self.dt

if __name__ == '__main__':

    rospy.init_node('thrusters_test')
    test = ThrusterTester(sys.argv)
    rospy.loginfo(rospy.get_name() + '[%s]: test init ')
    rospy.loginfo('Arguments are in order: thruster_id or (mulitple thrusters in pairs: 0, 1, 2), velocity step/frequency, time step, mode.\n'
                  'If not specified set to defaults: thruster_id=0, velocity_step=2, time_step=5, mode=linear.\n'
                  'Maximum frequency is 0.5Hz. Time step should be a multiple of 0.1. Mode can be linear, multiple or sine.\n')
    rospy.loginfo('Given args are: ' + str(sys.argv[1:]))
    test.run()

