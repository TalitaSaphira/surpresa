import rospy
import tf
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

class OdometryFusionClass:
    def __init__(self):
        # Odometria
        self.odom_msg = PoseWithCovarianceStamped()
        self.pos = np.array([0.0, 0.0])
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        # ROS
        rospy.init_node('Odometry_fusion')
        self.freq = 30

        self.wheel_odom_sub = rospy.Subscriber("/wheel/odom", Odometry, self.wheel_pose_cb)
        self.imu_odom_sub = rospy.Subscriber("/imu/data", Imu, self.imu_pose_cb)
        self.odom_pub = rospy.Publisher("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, queue_size=10)

    def wheel_pose_cb(self, msg):
        # Obtain position
        self.odom_msg.pose.pose.position = msg.pose.pose.position
        # self.pos = np.array([msg.pose.pose.position.x + self.origin_odom[0], msg.pose.pose.position.y + self.origin_odom[1]])

    def imu_pose_cb(self, msg):
        # Obtain orientation
        self.odom_msg.pose.pose.orientation = msg.orientation
        # self.quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

    def run(self):
        rate = rospy.Rate(self.freq)

        # Wait a bit
        rate.sleep()

        while not rospy.is_shutdown():
            self.odom_pub.publish(self.odom_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        x = OdometryFusionClass()
        x.run()
    except rospy.ROSInterruptException:
        pass
