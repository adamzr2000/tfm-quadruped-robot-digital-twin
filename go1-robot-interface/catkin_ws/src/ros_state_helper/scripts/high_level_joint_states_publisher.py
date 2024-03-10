#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from unitree_legged_msgs.msg import HighState  # Import the HighState message type
import threading

class CommandMapper:
    def __init__(self):
        rospy.init_node('go1_joint_states_publisher', anonymous=False)

        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.Subscriber("/high_state", HighState, self.high_state_callback)

        # Flag to indicate if high_state messages have been received
        self.received_high_state = False

        # Timer to periodically check for high_state messages
        self.check_timer = threading.Timer(1.0, self.check_high_state)
        self.check_timer.start()

    def high_state_callback(self, data):
        """
        Callback function for processing high-level state data from the physical robot.
        """
        # Indicate that high_state messages have been received
        self.received_high_state = True

        # Process and publish joint states based on high_state data
        topic = 'joint_states'
        rospy.loginfo("Number of motor state received: {}".format(len(data.motorState)))
        rospy.loginfo("Processing motor commands...")

        joint = JointState()
        joint.header = Header()
        joint.header.stamp = rospy.Time.now()
        joint.name = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 
                      'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 
                      'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 
                      'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']

        for i in range(12):
            joint.position.append(data.motorState[i].q)
            joint.velocity.append(data.motorState[i].dq)
            joint.effort.append(data.motorState[i].tauEst)
         
        self.pub.publish(joint)
        rospy.loginfo("Publishing to {}: \n {}".format(topic, joint))


    def check_high_state(self):
        """
        Checks if high_state messages are received.
        If not, publishes predefined joint_states.
        """
        if not self.received_high_state:
            rospy.loginfo("No high_state messages received. Publishing predefined joint states.")
            self.publish_predefined_joint_states()

        # If high_state messages are received, cancel the timer
        if self.received_high_state:
            self.check_timer.cancel()

        # Reset the timer only if high_state messages have not been received
        else:
            self.check_timer = threading.Timer(1.0, self.check_high_state)
            self.check_timer.start()

    def publish_predefined_joint_states(self):
        """
        Publishes predefined joint states.
        """
        joint = JointState()
        joint.header = Header()
        joint.header.stamp = rospy.Time.now()
        joint.name = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 
                      'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 
                      'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint', 
                      'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']
        joint.position = [0, 0.9, -1.853, 
                          0, 0.9, -1.853,  
                          0, 0.9, -1.853,  
                          0, 0.9, -1.853]
        self.pub.publish(joint)

    def shutdown_hook(self):
        """
        Called when the node is shutting down.
        Cancels the timer to avoid any residual calls.
        """
        self.check_timer.cancel()

if __name__ == '__main__':
    try:
        mapper = CommandMapper()
        rospy.on_shutdown(mapper.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




