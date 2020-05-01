#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from LAB3.msg import rtk_gps

if __name__ == '__main__':
    SENSOR_NAME = "gps"
    pub = rospy.Publisher(SENSOR_NAME + '/gps', rtk_gps, queue_size=10)
    rospy.init_node('gps_node', anonymous=True)

    serial_port = rospy.get_param('~port','/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate',115200)
    port = serial.Serial(serial_port, serial_baud)

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing rtk gps information.")

    try:
        while not rospy.is_shutdown():
            line = port.readline()
            if line.startswith('$GNGGA'):
                msg = rtk_gps()
                msg.header.stamp = rospy.Time.now()
                try:
                    all_info = line.split(',')
                    if all_info[3]=='N':
                        msg.latitude = int(all_info[2][0:2]) + float(all_info[2][2:])/60
                    else: msg.latitude = -1 * (int(all_info[2][0:2]) + float(all_info[2][2:])/60)
                    if all_info[5]=='E':
                        msg.longitude = int(all_info[4][0:3]) + float(all_info[4][3:])/60
                    else: msg.longitude = -1 * (int(all_info[4][0:3]) + float(all_info[4][3:])/60)
                    msg.altitude = float(all_info[9])
	            msg.fix = int(all_info[6]) # fix status
                    UTM = utm.from_latlon(msg.latitude, msg.longitude)
                    msg.utm_easting = UTM[0]
                    msg.utm_northing = UTM[1]
                    msg.zone_num = UTM[2]
                    msg.zone_letter = UTM[3]
                    print msg.latitude, msg.longitude, all_info[6], all_info[7], all_info[8]
                    pub.publish(msg)
                except:
                    rospy.logwarn("Data exception: " + line)
                    continue

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down rtk gps node...")
