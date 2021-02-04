#!/usr/bin/env python
# license removed for brevity
# -*- coding: utf-8 -*-

#############################################################################
# python ros vmu931
# by Kevin Chiu 2020
#############################################################################
from sensor_msgs.msg import Imu
import rospy
from std_msgs.msg import String

import numpy as np
import serial
import time
import sys
import struct

from std_msgs.msg import Float64

g_vmu_msg = Float64()


g_imu_msg = Imu()

############################################################################
# vmu Accelerometers reaad function
############################################################################
def VmuAccelerometersFuc():
    
    while 1:
        vmu_ser = np.zeros((50), dtype=np.int)
        loop = 0
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            if(input_data == 0x01):
                vmu_ser[loop] = input_data
                loop += 1
                break
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            vmu_ser[loop] = input_data
            loop += 1
            if(input_data == 0x04):
                break
        if(vmu_ser[2] == ord('q')):
            vmu_x = struct.unpack('>f', bytearray(list(vmu_ser[7:11])))[0]
            vmu_y = struct.unpack('>f', bytearray(list(vmu_ser[11:15])))[0]
            vmu_z = struct.unpack('>f', bytearray(list(vmu_ser[15:19])))[0]
            if(vmu_x != 0.0 and vmu_y != 0.0 and vmu_z != 0.0):
                break

    return vmu_x, vmu_y, vmu_z

############################################################################
# vmu Gyroscopes reaad function
############################################################################
def VmuGyroscopesFuc():
    
    while 1:
        vmu_ser = np.zeros((50), dtype=np.int)
        loop = 0
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            if(input_data == 0x01):
                vmu_ser[loop] = input_data
                loop += 1
                break
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            vmu_ser[loop] = input_data
            loop += 1
            if(input_data == 0x04):
                break
        if(vmu_ser[2] == ord('q')):
            vmu_x = struct.unpack('>f', bytearray(list(vmu_ser[7:11])))[0]
            vmu_y = struct.unpack('>f', bytearray(list(vmu_ser[11:15])))[0]
            vmu_z = struct.unpack('>f', bytearray(list(vmu_ser[15:19])))[0]
            if(vmu_x != 0.0 and vmu_y != 0.0 and vmu_z != 0.0):
                break

    return vmu_x, vmu_y, vmu_z

############################################################################
# vmu Quaternions reaad function
############################################################################
def VmuQuaternionsFuc():
    
    while 1:
        vmu_ser = np.zeros((50), dtype=np.int)
        loop = 0
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            if(input_data == 0x01):
                vmu_ser[loop] = input_data
                loop += 1
                break
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            vmu_ser[loop] = input_data
            loop += 1
            if(input_data == 0x04):
                break
        if(vmu_ser[2] == ord('q')):
            vmu_w = struct.unpack('>f', bytearray(list(vmu_ser[7:11])))[0]
            vmu_x = struct.unpack('>f', bytearray(list(vmu_ser[11:15])))[0]
            vmu_y = struct.unpack('>f', bytearray(list(vmu_ser[15:19])))[0]
            vmu_z = struct.unpack('>f', bytearray(list(vmu_ser[19:23])))[0]
            if(vmu_w != 0.0 and vmu_x != 0.0 and vmu_y != 0.0 and vmu_z != 0.0):
                break

    return vmu_w, vmu_x, vmu_y, vmu_z

############################################################################
# vmu Euler reaad function
############################################################################
def VmuEulerFuc():

    while 1:
        vmu_ser = np.zeros((50), dtype=np.int)
        loop = 0
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            if(input_data == 0x01):
                vmu_ser[loop] = input_data
                loop += 1
                break
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
                # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            vmu_ser[loop] = input_data
            loop += 1
            if(input_data == 0x04):
                break
        if(vmu_ser[2] == ord('e')):
            vmu_x = struct.unpack('>f', bytearray(list(vmu_ser[7:11])))[0]
            vmu_y = struct.unpack('>f', bytearray(list(vmu_ser[11:15])))[0]
            vmu_z = struct.unpack('>f', bytearray(list(vmu_ser[15:19])))[0]
            if(vmu_x != 0.0 and vmu_y != 0.0 and vmu_z != 0.0):
                break

    return vmu_x, vmu_y, vmu_z

##############################################################################
# vmu talker node
###########################################################################
def vmu_talker():
    global g_vmu_msg
    global g_imu_msg

    # serial_work = threading.Thread(target=publisher_thread)
    # serial_work.start()

    pub = rospy.Publisher('imu', Imu, queue_size=1000)
    rospy.init_node('vmu_talker', anonymous=True)
    # rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        acc_x, acc_y, acc_z = VmuAccelerometersFuc()
        gyro_x, gyro_y, gyro_z = VmuGyroscopesFuc()
        quate_w, quate_x, quate_y, quate_z = VmuQuaternionsFuc()
        euler_x, euler_y, euler_z = VmuEulerFuc()
        # print('euler_x:%.4f euler_y:%.4f euler_z:%.4f'%(euler_x, euler_y, euler_z))

        g_vmu_msg = euler_z
        print(g_vmu_msg)

        g_imu_msg.header.stamp = rospy.Time.now()
        g_imu_msg.header.frame_id = "base_link"
        g_imu_msg.orientation.x = quate_x
        g_imu_msg.orientation.y = quate_y
        g_imu_msg.orientation.z = quate_z
        g_imu_msg.orientation.w = quate_w
        g_imu_msg.linear_acceleration.x = acc_x
        g_imu_msg.linear_acceleration.y = acc_y
        g_imu_msg.linear_acceleration.z = acc_z
        g_imu_msg.angular_velocity.x = gyro_x
        g_imu_msg.angular_velocity.y = gyro_y
        g_imu_msg.angular_velocity.z = gyro_z

        pub.publish(g_imu_msg)

        # rate.sleep()


############################################################################
# main
#############################################################################
if __name__ == '__main__':
    print("vmu start......")
    ################input ############################
    try:
        input_argv = sys.argv
        input_port = input_argv[1]
        input_baudrate = input_argv[2]
        print('====== input setting ======')
    ################# defalt #######################
    except:
        input_port = "/dev/ttyACM0"
        input_baudrate = "115200"
        print('====== defalt setting ======')
    ################# serial connect ################
    print("port: " + input_port)
    print("baudrate: " + input_baudrate)
    print('=========================')
    try:
        g_ser_vmu = serial.Serial(input_port, input_baudrate, bytesize=8,
                                  parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)
    except:
        print('\033[91m' + "serial error!!!")
        # rospy.loginfo()
        while 1:
            continue
    time.sleep(1)

    ################### test serial #################
    if(g_ser_vmu.read().encode('hex') == ""):
        print('\033[91m' + "serial error!!!")
        # rospy.loginfo()
        while 1:
            continue

    try:
        vmu_talker()
    except rospy.ROSInterruptException:
        pass
