#!/usr/bin/env python3

import serial
import time
import os
import math

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse

class ArmDriver(object):
    def __init__(self):

        usb_port = os.getenv("ARM_PORT")
        if usb_port is None:
            usb_port = "/dev/ttyACM0"
        try:
            self.board = serial.Serial(usb_port, 115200, rtscts=True)
        except serial.serialutil.SerialException: # type: ignore
            raise(Exception(f"Can't find port {usb_port}"))
        
        self.joint_state_pub = rospy.Publisher("/AiForAll/manipulation/joint_states", JointState, queue_size=1)
        self.go_to_jointstate_sub = rospy.Subscriber("/AiForAll/manipulation/go_joint_states", JointState, self.go_to_jointstate_cb)

    def read_angle(self):
        pkg = b"#RA0|"
        self.board.write(pkg)
        time.sleep(0.2)
        if self.board.in_waiting > 0:
            data = self.board.read_all()
            data = data.decode() # type: ignore
            data = data.split("\r\n")
            data = data[1].split(" ")
            try:
                for i in range(len(data)):
                    data[i] = float(data[i]) # type: ignore
            except ValueError:
                return 1
            message = JointState()
            message.header.stamp = rospy.Time.now()
            message.name = ["motor_1", "motor_2", "motor_3"]
            message.position = data
            self.joint_state_pub.publish(message)
            # print(data)
        
    def go_to_jointstate_cb(self, data):
        # home_position = [0.1607, 4.0982, 9.0804]
        # up_position = [25.3929, 12.3750, 30.2143]
        position = list(data.position)
        for i in range(len(position)):
            position[i] = str(position[i] * 180 / math.pi)
        position = " ".join(position)
        pkg = f"#AA3 {position}|".encode()
        rospy.logdebug(pkg)
        self.board.write(pkg)
    
    def start(self):
        while not rospy.is_shutdown():
            self.read_angle()
            


if __name__ == "__main__":
    rospy.init_node("aiforall_manipulation", anonymous=True, log_level=rospy.DEBUG)
    driver = ArmDriver()
    driver.start()

