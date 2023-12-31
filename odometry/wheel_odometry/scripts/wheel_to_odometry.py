#!/usr/bin/env python3

"""
Universidade Federal de Minas Gerais (UFMG) - 2023
Laboratorio CORO
Contact:
Joao Baiao, <baiaojfr.eng@gmail.com>
"""

import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
import numpy as np


class WheelOdometry:
    def __init__(self):
        rospy.init_node('wheel_odometry_node')

        self.listener = tf.TransformListener()

        self.freq = 30
        self.time_last = rospy.Time.now().to_sec()
        self.time_step = 0.1
        
        # Wheel velocities array
        self.u = np.array([0.0, 0.0, 0.0, 0.0])

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.quaternion_imu = np.array([1.0, 0.0, 0.0, 0.0])

        self.read_params()

        self.odom_sub = rospy.Subscriber(self.odometry_topic_name, Odometry, self.odom_callback)
        self.vel_sub = rospy.Subscriber('/wheel_velocities', Float32MultiArray, self.velocity_callback)
        self.wheel_odom_pub = rospy.Publisher('/wheel/odom', Odometry, queue_size=10)

    def read_params(self):
        # Obtain the parameters
        # try:
        self.l = float(rospy.get_param("~l", 1.0))
        self.w = float(rospy.get_param("~w", 1.0)) 
        self.r = float(rospy.get_param("~r", 1.0)) 

        self.odometry_topic_name = (rospy.get_param("~odometry_topic_name", "gt")) 

    def odom_callback(self, msg):
        aux = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]

        # Calculate odometry based on wheels velocities vector u
        self.quaternion_imu = np.array(aux)


    def velocity_callback(self, msg):
        # Calculate odometry based on wheels velocities vector u
        self.u = np.array(msg.data)

        # Estimate time step
        time_now = rospy.Time.now().to_sec()
        self.time_step = time_now - self.time_last # MELECA pode ser q o primeiro seja comicamente grande
        self.time_last = time_now

    def run(self):
        rate = rospy.Rate(self.freq)

        # Wait a bit
        rate.sleep()


        while not rospy.is_shutdown():
            # Direct Cinematics
            M = [[1, 1, 1, 1],[-1, 1, -1, 1],[-1/(self.l + self.w), 1/(self.l + self.w), 1/(self.l + self.w), -1/(self.l + self.w)]]
            [x_dot_b,y_dot_b,theta_dot] = (self.r/4) * (M @ self.u)


            rot_bw = R.from_quat(self.quaternion_imu)
            [x_dot, y_dot, z_dot] = rot_bw.apply(np.array([x_dot_b, y_dot_b, 0.0]))

            # Apply translation
            self.x += x_dot*self.time_step
            self.y += y_dot*self.time_step
            
            rot_dot = R.from_euler('x', theta_dot*self.time_step)
            rot = R.from_quat(self.quaternion_imu)

            # Apply rotation 
            rot_new = rot_dot*rot
            self.quaternion = rot_new.as_quat()

            # Create an Odometry message and publish it
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = "wheel_odom"
            odom_msg.child_frame_id = "base_footprint"

            # Set the position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            # Convert theta to quaternion and set the orientation
            odom_msg.pose.pose.orientation.w = self.quaternion[0]
            odom_msg.pose.pose.orientation.x = self.quaternion[1]
            odom_msg.pose.pose.orientation.y = self.quaternion[2]
            odom_msg.pose.pose.orientation.z = self.quaternion[3]

            # Set the twist (velocity) information
            odom_msg.twist.twist.linear.x = 0.0 # MELECA pode ser q tenha q publicar ai seria o x_dot*time_step etc
            odom_msg.twist.twist.angular.z = 0.0 

            odom_msg.pose.covariance = [0.210, 9.6136e-05, 6.8828e-05, 0, 0, 0,
                                    9.6136e-05, 0.33, 5.5846e-05, 0, 0, 0,
                                    6.8828e-05, 5.5846e-05, 8.4270e-04, 0, 0, 0,
                                    0, 0, 0, 99999, 0, 0,
                                    0, 0, 0, 0, 99999, 0,
                                    0, 0, 0, 0, 0, 99999] # MELECA kkkk

            # Publish the odometry message
            self.wheel_odom_pub.publish(odom_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        odometry = WheelOdometry()
        odometry.run()
    except rospy.ROSInterruptException:
        pass
