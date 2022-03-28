#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import csv
import time
import rospkg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import AttitudeTarget


class ThrustCalibration:

    def __init__(self):

        # Load parameters
        self.time_average_interval = float(
            rospy.get_param('~time_interval', 1.0))
        self.vbat_min = float(
            rospy.get_param('~min_battery_voltage', 13.2))
        self.mass_kg = float(
            rospy.get_param('~mass_kg', 1.0))

        # store measurements of vbat and commanded thrusts
        self.volt_buf = np.array([])
        self.volt_records = np.array([])
        self.cmd_buf = np.array([])
        self.cmd_records = np.array([])
        self.last_records_t = rospy.get_rostime()

        # Some flags
        self.count_start_trigger_received = 0

        # Subscribe
        self.bat_sub = rospy.Subscriber(
            "/mavros/battery", BatteryState, self.battery_voltage_cb)
        rospy.Subscriber(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, self.thrust_commands_cb)
        rospy.Subscriber(
            "/traj_start_trigger", PoseStamped, self.start_trigger_cb)

    def battery_voltage_cb(self, msg):

        if (self.count_start_trigger_received == 0):
            return

        self.volt_buf = np.append(
            self.volt_buf, np.array([msg.voltage]), axis=0)

        # Record filted data
        cur_t = rospy.get_rostime()
        if ((cur_t - self.last_records_t).to_sec() > self.time_average_interval):
            self.last_records_t = cur_t
            # print self.volt_records.shape
            # print self.volt_buf.shape
            # print
            # print np.mean(self.volt_buf)
            self.volt_records = np.append(
                self.volt_records, np.array([np.mean(self.volt_buf)]), axis=0)
            self.volt_buf = np.array([])
            self.cmd_records = np.append(
                self.cmd_records, np.array([np.mean(self.cmd_buf)]), axis=0)
            self.cmd_buf = np.array([])
            print "volt=", self.volt_records[-1], " thr=", self.cmd_records[-1]

        # Calculate calibration parameters
        if ((self.count_start_trigger_received >= 2) or 
            (self.volt_records.size != 0 and self.volt_records[-1] < self.vbat_min)):
            self.cal_and_save_data()
            self.bat_sub.unregister()

    def thrust_commands_cb(self, msg):

        if (self.count_start_trigger_received == 0):
            return

        self.cmd_buf = np.append(self.cmd_buf, np.array([msg.thrust]), axis=0)
        print msg.thrust

    def start_trigger_cb(self, msg):

        self.count_start_trigger_received += 1
        if(self.count_start_trigger_received == 1):
            rospy.loginfo("Start recording.")
        if(self.count_start_trigger_received > 1):
            rospy.loginfo("Stop recording.")

    def cal_and_save_data(self):
        rospy.loginfo("Data storing.")

        data_stack = np.vstack((self.cmd_records, self.volt_records))
        data = [tuple(x) for x in data_stack.tolist()]

        rospack = rospkg.RosPack()
        filedir = rospack.get_path(
            'px4ctrl')+'/thrust_calibrate_scrips/data.csv'
        f = open(filedir, 'a')
        writer = csv.writer(f)
        writer.writerow(((time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())),
                         'mass(kg):', self.mass_kg, 'commands', 'voltage'))
        writer.writerows(data)
        f.close()

        rospy.loginfo("Stored to " + filedir)


if __name__ == '__main__':

    try:
        rospy.init_node("thrust_calibration")

        thrust_calibration = ThrustCalibration()
        rospy.loginfo("Waiting for trigger.")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
