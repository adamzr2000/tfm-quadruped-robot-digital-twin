#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
from unitree_legged_msgs.msg import HighState  
import threading

class CommandMapper:
    def __init__(self):
        rospy.init_node('go1_imu_publisher', anonymous=False)

        self.imu_pub = rospy.Publisher('trunk_imu', Imu, queue_size=1)  

        # Subscribe to the /high_state topic to receive data from the physical robot
        rospy.Subscriber("/high_state", HighState, self.high_state_callback)


    def high_state_callback(self, data):
        """
        Callback function for processing high-level state data from the physical robot.
        """

        # Process and publish IMU data based on high_state data
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "imu_link"

        imu.orientation = Quaternion(data.imu.quaternion[1], data.imu.quaternion[2],
                                     data.imu.quaternion[3], data.imu.quaternion[0])

        imu.angular_velocity = Vector3(data.imu.gyroscope[0], data.imu.gyroscope[1], data.imu.gyroscope[2])
        imu.linear_acceleration = Vector3(data.imu.accelerometer[0], data.imu.accelerometer[1], data.imu.accelerometer[2])

        # Populate covariance matrices (set to all zeros for unknown covariance)
        imu.orientation_covariance = [0.0] * 9
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9
        self.imu_pub.publish(imu)
        rospy.loginfo("Publishing to {}: \n {}".format('trunk_imu', imu))



if __name__ == '__main__':
    try:
        # Instantiate CommandMapper and initiate ROS communication loop
        mapper = CommandMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
