#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Point
from unitree_legged_msgs.msg import HighState

class CommandMapper:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('go1_odom_publisher', anonymous=False)

        # Publishers for odometry and TF
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        # Subscribe to the /high_state topic to receive data from the physical robot
        rospy.Subscriber("/high_state", HighState, self.high_state_callback)


    def high_state_callback(self, data):
        """
        Callback function for processing high-level state data from the physical robot.
        """

        current_time = rospy.Time.now()

        # Transform broadcaster for publishing TF data
        br = tf.TransformBroadcaster()

        br.sendTransform(
            translation=(data.position[0], data.position[1], data.position[2]),
            rotation=(data.imu.quaternion[0], data.imu.quaternion[1], data.imu.quaternion[2], data.imu.quaternion[3]),
            time=current_time,
            child="base",
            parent="odom"
        )

        # Prepare and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base"

        # Populate odometry message with position, orientation, velocity, and gyroscope data
        odom.pose.pose.position = Point(data.position[0], data.position[1], data.position[2])
        odom.pose.pose.orientation = Quaternion(data.imu.quaternion[0], data.imu.quaternion[1],
                                     data.imu.quaternion[2], data.imu.quaternion[3])
        odom.pose.covariance = [1e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 1e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-09] 

        odom.twist.twist.linear = Vector3(data.velocity[0], data.velocity[1], data.velocity[2])
        odom.twist.twist.angular = Vector3(data.imu.gyroscope[0], data.imu.gyroscope[1], data.imu.gyroscope[2])
        odom.twist.covariance = [1e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 1e-09, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-09]
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        # Log information about published odometry data
        rospy.loginfo("Publishing to {}: \n {}".format('odom', odom))

if __name__ == '__main__':
    try:
        # Instantiate CommandMapper and initiate ROS communication loop
        mapper = CommandMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
