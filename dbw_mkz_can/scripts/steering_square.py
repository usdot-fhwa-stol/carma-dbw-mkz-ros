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
import csv
from std_msgs.msg import Empty
from dbw_mkz_msgs.msg import SteeringCmd, SteeringReport
from dbw_mkz_msgs.msg import GearReport
import math
import os

class SteeringSquare:
    def __init__(self):
        # TO-DO: Add a parameter to change the rate of oscillation?
        # TO-DO: Add a parameter to change the output file path?
        
        rospy.init_node('steering_square')

        # Variables for logging
        self.steering_cmd = 0.0
        self.msg_steering_report = SteeringReport()
        self.msg_steering_report_ready = False

        # Other drive-by-wire variables
        self.msg_gear_report = GearReport()
        self.msg_gear_report_ready = False

        # Parameters
        self.time_current = 0.00
        self.param_start_delay = rospy.get_param("~start_delay", 2.00) # Delay in seconds before the script starts.
        self.param_duration = rospy.get_param("~duration", 30.000)     # Duration in seconds.
        self.param_resolution = rospy.get_param("~resolution", 0.050)  # Time between recording values.
        self.param_minimum = rospy.get_param("~minimum", -2.000)       # Minimum (leftmost) steering value, when oscillating.
        self.param_maximum = rospy.get_param("~maximum", 2.000)        # Maximum (rightmost) steering value, when oscillating.
        self.range = self.param_maximum - self.param_minimum
        self.median = 0

        rospy.loginfo('Recording steering wheel data every ' + "{:.03f}".format(self.param_resolution) + ' seconds from 0.000 to '
                        + "{:.03f}".format(self.param_duration) + ' with ' + "{:.03f}".format(self.param_resolution) + ' increments.')
        rospy.loginfo('This will take ' + "{:.03f}".format(self.param_duration / 60.0) + ' minutes.')

        # Open CSV file
        self.csv_file = open('steering_square_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file, delimiter=',')
        self.csv_writer.writerow(['Time (s)', 'Steering Cmd', 'Steering Actual', 'Torque (Nm)'])
        rospy.loginfo('Recording to file ' + str(os.path.abspath("steering_square_data.csv")))

        # Publishers and subscribers
        self.pub_steering = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        rospy.Subscriber('/vehicle/gear_report', GearReport, self.recv_gear)
        rospy.Subscriber('/vehicle/steering_report', SteeringReport, self.recv_steering)
        # Periodically send messages
        rospy.Timer(rospy.Duration(0.01), self.timer_cmd)

        # Send an empty message to start the DBW system.
        self.pub_enable = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
        # Wait until we're connected (check 10 times a second) before sending a message.
        r = rospy.Rate(10)
        while self.pub_enable.get_num_connections() == 0:
            r.sleep()
        # Publish a message, informing the DBW system it should be enabled.
        self.pub_enable.publish(Empty())
        
        # Wait for the system to start up before starting the script.
        rospy.sleep(self.param_start_delay)
        # Periodically receive/record messages
        rospy.Timer(rospy.Duration(self.param_resolution), self.timer_process)

    def timer_process(self, event):
        if self.time_current == 0.00:
            # Check for safe conditions
            if not self.msg_steering_report_ready:
                rospy.logerr('Speed check failed. No messages on topic \'/vehicle/steering_report\'')
                rospy.signal_shutdown('')
            if self.msg_steering_report.speed > 1.0:
                rospy.logerr('Speed check failed. Vehicle speed is greater than 1 m/s.')
                rospy.signal_shutdown('')
            if not self.msg_gear_report_ready:
                rospy.logerr('Gear check failed. No messages on topic \'/vehicle/gear_report\'')
                rospy.signal_shutdown('')
            if not self.msg_gear_report.state.gear == self.msg_gear_report.state.PARK:
                rospy.logerr('Gear check failed. Vehicle not in park.')
                rospy.signal_shutdown('')
        elif self.time_current < self.param_duration:
            # Check for new messages
            if not self.msg_steering_report_ready:
                rospy.logerr('No new messages on topic \'/vehicle/steering_report\'')
                rospy.signal_shutdown('')
            if not self.msg_steering_report.enabled:
                rospy.logerr('Steering module not enabled!:' + str(self.steering_cmd))
                rospy.signal_shutdown('')
            
            rospy.loginfo('Data point: ' + "{:.03f}".format(self.time_current)                  + ', ' +
                                           "{:.03f}".format(self.msg_steering_report.steering_wheel_cmd)    + ', ' +
                                           "{:.03f}".format(self.msg_steering_report.steering_wheel_angle) + ', ' +
                                           str(self.msg_steering_report.steering_wheel_torque))
            self.csv_writer.writerow(["{:.03f}".format(self.time_current),
                                      "{:.03f}".format(self.msg_steering_report.steering_wheel_cmd),
                                      "{:.03f}".format(self.msg_steering_report.steering_wheel_angle),
                                      str(self.msg_steering_report.steering_wheel_torque)])
        else:
            rospy.loginfo("Shutting down, loop finish...")
            rospy.signal_shutdown('')
        
        # Prepare for next iteration
        self.time_current += self.param_resolution
        self.msg_steering_report_ready = False
        self.msg_gear_report_ready = False
        # square alternates between min and max, 
        rospy.loginfo("VALUE:" + str((self.range / 2.0)))
        self.steering_cmd = self.param_maximum if self.time_current % 2 > 1 else self.param_minimum

    def timer_cmd(self, event):
        if self.time_current < self.param_duration:
            msg = SteeringCmd()
            msg.enable = True
            msg.cmd_type = SteeringCmd.CMD_ANGLE
            msg.steering_wheel_angle_cmd = self.steering_cmd
            self.pub_steering.publish(msg)

    def recv_gear(self, msg):
        self.msg_gear_report = msg
        self.msg_gear_report_ready = True

    def recv_steering(self, msg):
        self.msg_steering_report = msg
        self.msg_steering_report_ready = True

    def shutdown_handler(self):
        rospy.loginfo('Saving csv file')
        # Disable brake control system.
        self.steering_cmd = 0.0
        msg = SteeringCmd()
        msg.enable = False
        msg.steering_wheel_angle_cmd = 0.0
        self.pub_steering.publish(msg)
        # Disable DBW control system.
        self.pub_disable = rospy.Publisher('/vehicle/disable', Empty, queue_size=1)
        self.pub_disable.publish(Empty())
        self.csv_file.close()


if __name__ == '__main__':
    try:
        node = SteeringSquare()
        rospy.on_shutdown(node.shutdown_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
