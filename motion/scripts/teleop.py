#!/usr/bin/env python3

"""
Universidade Federal de Minas Gerais (UFMG) - 2023
Laboratorio CORO
Contact:
Joao Baiao, <baiaojfr.eng@gmail.com>
"""

# Teleoperação de robô ominidirecional por meio de um jostick

import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation as R
from Navigator_class import Navigation_class
import numpy as np


class Teleoperator:
    def __init__(self):
        rospy.init_node('Teleoperator_Node')
        self.teleop_sub = rospy.Subscriber("/joy", Joy, self.callback_joy)
        self.navigator = Navigation_class()
        self.navigator.read_params()

        self.desired_velocity_b = [0.0,0.0]
        self.freq = 30
        self.yaw = 0.0

    def callback_joy(self, msg):
        change_x = msg.axes[1] + msg.buttons[3] - msg.buttons[1]
        change_y = msg.axes[0] + msg.buttons[0] - msg.buttons[2]

        self.yaw = 0.2*msg.axes[3]

        self.desired_velocity_b = [0.1*change_x, 0.1*change_y]


    def run(self):
        rate = rospy.Rate(self.freq)

        rate.sleep()

        while not rospy.is_shutdown():
            self.navigator.setMovement(self.desired_velocity_b[:2], self.yaw)

            rate.sleep()

if __name__ == '__main__':
    try:
        navegator = Teleoperator()
        navegator.run()
    except rospy.ROSInterruptException:
        pass
