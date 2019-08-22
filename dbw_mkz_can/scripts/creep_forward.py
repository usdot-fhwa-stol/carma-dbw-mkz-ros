#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014-2018, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from dbw_mkz_msgs.msg import BrakeCmd, BrakeReport, BrakeInfoReport,\
    ThrottleCmd, ThrottleReport, ThrottleInfoReport, GearCmd, GearReport,\
    SteeringCmd, SteeringReport
import std_msgs

class CreepForward:
"""
Test that oscillates between throttle and brake commands to test edge cases that occur during stop/starting
"""
    def __init__(self):
        rospy.init_node('creep_forward')

        # Variables for logging
        self.brake_cmd = 0.25
        self.msg_brake_report = BrakeReport()
        self.msg_brake_report_ready = False
        self.msg_brake_info_report = BrakeInfoReport()
        self.msg_brake_info_report_ready = False
        self.throttle_cmd = 0.25
        self.msg_throttle_report = ThrottleReport()
        self.msg_throttle_report_ready = False
        self.msg_throttle_info_report = ThrottleInfoReport()
        self.msg_throttle_info_report_ready = False

        # Other drive-by-wire variables
        self.msg_gear_report = GearReport()
        self.msg_gear_report_ready = False
        self.msg_steering_report = SteeringReport()
        self.msg_steering_report_ready = False
        self.vehicle_enabled = False

        # Parameters
        self.i = 0
        self.test = 0
        self.resolution = 1
        self.duration = 20
        self.initializing = True

        # Publishers and subscribers
        self.bpub = rospy.Publisher(
            '/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.tpub = rospy.Publisher(
            '/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.gpub = rospy.Publisher(
            '/vehicle/gear_cmd', GearCmd, queue_size=1)
        self.spub = rospy.Publisher(
            '/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.epub = rospy.Publisher(
            '/vehicle/enable', std_msgs.msg.Empty, queue_size=1)
        rospy.Subscriber('/vehicle/brake_report',
                         BrakeReport, self.recv_brake)
        rospy.Subscriber('/vehicle/brake_info_report',
                         BrakeInfoReport, self.recv_brake_info)
        rospy.Subscriber('/vehicle/throttle_report',
                         ThrottleReport, self.recv_throttle)
        rospy.Subscriber('/vehicle/throttle_info_report',
                         ThrottleInfoReport, self.recv_throttle_info)
        rospy.Subscriber('/vehicle/gear_report',
                         GearReport, self.recv_gear)
        rospy.Subscriber('/vehicle/steering_report',
                         SteeringReport, self.recv_steering)
        rospy.Subscriber('/vehicle/dbw_enabled',
                         std_msgs.msg.Bool, self.recv_enabled)
        rospy.loginfo('Testing brake creep corner case')
        rospy.Timer(rospy.Duration(0.02), self.timer_cmd)
        rospy.Timer(rospy.Duration(self.resolution), self.timer_process)
        self.continue_after_user()

    def continue_after_user(self):
        self.initializing = True
        rospy.Timer(rospy.Duration(0.5), self.initialize, oneshot=True)

    def initialize(self, event):
        raw_input('shift into park and press Enter to continue testing')
        # Check for safe conditions
        if not self.msg_steering_report_ready:
            rospy.logerr(
            "Speed check failed. No messages on topic '/vehicle/steering_report'")
            rospy.signal_shutdown('')
            return
        if self.msg_steering_report.speed > 1.0:
            rospy.logerr(
            'Speed check failed. Vehicle speed is greater than 1 m/s.')
            rospy.signal_shutdown('')
            return
        if not self.msg_gear_report_ready:
            rospy.logerr(
            "Gear check failed. No messages on topic '/vehicle/gear_report'")
            rospy.signal_shutdown('')
            return
        if self.msg_gear_report.state.gear != self.msg_gear_report.state.PARK:
            rospy.logerr('Gear check failed. Vehicle not in park, it is in ' + str(self.msg_gear_report.state.gear))
            rospy.signal_shutdown('')
            return
        rospy.loginfo('Shifting into drive')
        timeout = 10.0
        wait_time = 0.0
        while not self.vehicle_enabled and wait_time < timeout:
            self.epub.publish()
            rospy.sleep(0.02)
            wait_time += 0.02
        while not self.msg_gear_report.state.gear == self.msg_gear_report.state.DRIVE and wait_time < timeout:
            gmsg = GearCmd()
            gmsg.cmd.gear = gmsg.cmd.DRIVE
            self.gpub.publish(gmsg)
            rospy.sleep(0.02)
            wait_time += 0.02
        if wait_time < timeout:
            rospy.loginfo('Continuing test')
            self.initializing = False
        else:
            rospy.logerr('Unable to shift into gear')
            rospy.signal_shutdown('')

    def timer_process(self, event):
        if self.initializing:
            return
        # Check for new messages
        if not self.msg_brake_report_ready:
            rospy.logerr(
                "No new messages on topic '/vehicle/brake_report'")
            rospy.signal_shutdown('')
        if not self.msg_brake_info_report_ready:
            rospy.logerr(
                "No new messages on topic '/vehicle/brake_info_report'")
            rospy.signal_shutdown('')
        if not self.msg_brake_report.enabled:
            rospy.logerr('Brake module not enabled!')
            rospy.signal_shutdown('')
        if not self.msg_throttle_report_ready:
            rospy.logerr(
                "No new messages on topic '/vehicle/throttle_report'")
            rospy.signal_shutdown('')
        if not self.msg_throttle_info_report_ready:
            rospy.logerr(
                "No new messages on topic '/vehicle/throttle_info_report'")
            rospy.signal_shutdown('')
        if not self.msg_throttle_report.enabled:
            rospy.logerr('Throttle module not enabled!')
            rospy.signal_shutdown('')
        if self.msg_brake_report.pedal_input > 0.19:
            rospy.logwarn(
                'Take your foot off the brake pedal!')

        if self.i == self.duration:
            # change to another brake/throttle setting
            self.i = 0
            self.test += 1
            if self.throttle_cmd == 1.0:
                self.throttle_cmd = 0.25
                self.brake_cmd += 0.25
            else:
                self.throttle_cmd += 0.25
            rospy.loginfo('Progress %.2f%%' % (self.test / 0.01))
            rospy.loginfo('Testing brake=%f, throttle=%f' %
                            (self.brake_cmd, self.throttle_cmd))
            # check to see if we are finished
            if self.brake_cmd > 1.0:
                rospy.loginfo('Finished testing!')
                rospy.signal_shutdown('')
            else:
                self.continue_after_user()
        self.i += 1
        self.msg_brake_report_ready = False
        self.msg_brake_info_report_ready = False
        self.msg_throttle_report_ready = False
        self.msg_throttle_info_report_ready = False

    def timer_cmd(self, event):
        if self.initializing:
            return
        bmsg = BrakeCmd()
        tmsg = ThrottleCmd()
        tmsg.enable = bmsg.enable = True
        bmsg.pedal_cmd_type = BrakeCmd.CMD_PERCENT
        tmsg.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        # brake when we are moving and throttle when we are stopped
        # the goal here is to alternate between stopped and moving states as
        # quickly as possible
        if self.msg_steering_report.speed > 0:
            bmsg.pedal_cmd = self.brake_cmd
            tmsg.pedal_cmd = 0
        else:
            bmsg.pedal_cmd = 0
            tmsg.pedal_cmd = self.throttle_cmd
        self.bpub.publish(bmsg)
        self.tpub.publish(tmsg)

    def recv_brake(self, msg):
        self.msg_brake_report = msg
        self.msg_brake_report_ready = True

    def recv_brake_info(self, msg):
        self.msg_brake_info_report = msg
        self.msg_brake_info_report_ready = True

    def recv_throttle(self, msg):
        self.msg_throttle_report = msg
        self.msg_throttle_report_ready = True

    def recv_throttle_info(self, msg):
        self.msg_throttle_info_report = msg
        self.msg_throttle_info_report_ready = True

    def recv_gear(self, msg):
        self.msg_gear_report = msg
        self.msg_gear_report_ready = True

    def recv_enabled(self, msg):
        self.vehicle_enabled = msg.data

    def recv_steering(self, msg):
        self.msg_steering_report = msg
        self.msg_steering_report_ready = True

    def shutdown_handler(self):
        if self.test < 100:
            rospy.logerr('Failure during testing')


if __name__ == '__main__':
    try:
        node = CreepForward()
        rospy.on_shutdown(node.shutdown_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
