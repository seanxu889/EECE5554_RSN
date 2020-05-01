#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import tf
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

if __name__ == '__main__':
    SENSOR_NAME = "imu"
    pub = rospy.Publisher(SENSOR_NAME + '/imu', Imu, queue_size=10)
    pub_mag = rospy.Publisher(SENSOR_NAME + '/imu_mag', MagneticField, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)

    serial_port = rospy.get_param('~port','/dev/ttyUSB1')
    serial_baud = rospy.get_param('~baudrate',115200)
    port = serial.Serial(serial_port, serial_baud)

    rospy.logdebug("Initialization complete")
    rospy.loginfo("Publishing imu information.")

    try:
        while not rospy.is_shutdown():
            line = port.readline()
            # eg: $VNYMR,-078.104,+005.642,+023.096,+00.0125,+00.1252,-00.1042,+00.878,-03.762,-08.854,-00.010387,-00.028062,-00.061305*68
            if line.startswith('$VNYMR'):
                MSG_a = Imu()
                MSG_mag = MagneticField()
                MSG_a.header.stamp = rospy.Time.now()
                MSG_mag.header.stamp = rospy.Time.now()
                try:
                    all_info = line.split(',')

                    Y = np.deg2rad(float(all_info[1]))
                    P = np.deg2rad(float(all_info[2]))
                    R = np.deg2rad(float(all_info[3]))
                    quaternions = tf.transformations.quaternion_from_euler(R, P, Y) # roll, pitch, yaw
                    MSG_a.orientation.x = quaternions[0]
                    MSG_a.orientation.y = quaternions[1]
                    MSG_a.orientation.z = quaternions[2]
                    MSG_a.orientation.w = quaternions[3]

                    MSG_a.linear_acceleration.x = float(all_info[7])
                    MSG_a.linear_acceleration.y = float(all_info[8])
                    MSG_a.linear_acceleration.z = float(all_info[9])

                    MSG_a.angular_velocity.x = float(all_info[10])
                    MSG_a.angular_velocity.y = float(all_info[11])
                    MSG_a.angular_velocity.z = float(all_info[12][0:10])

                    MSG_mag.magnetic_field.x = float(all_info[4])
                    MSG_mag.magnetic_field.y = float(all_info[5])
                    MSG_mag.magnetic_field.z = float(all_info[6])

                    print MSG_a.orientation.x, MSG_a.linear_acceleration.x, MSG_a.angular_velocity.x, MSG_mag.magnetic_field.x
                    pub.publish(MSG_a)
                    pub_mag.publish(MSG_mag)
                except:
                    rospy.logwarn("Data exception: " + line)
                    continue
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        port.close()

    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down imu node...")
