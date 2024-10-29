#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math  # Import math to handle wrapping

class JointStatePublisher:
    def __init__(self):
        rospy.init_node('joint_state_publisher_node', anonymous=True)

        # Publisher for joint states
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Set the publishing rate (10 Hz)
        self.rate = rospy.Rate(10)

        # Define joint names for all your joints
        self.joint_names = [
            # Fixed joints
            'base_to_chassis',
            'joint_chassis_to_cylinder_left',
            'joint_chassis_to_cylinder_right',

            # Revolute joints
            'joint_cylinder_left_to_rocker_left',
            'joint_cylinder_left_to_rocker_right',
            'joint_rocker_to_bogie_left',
            'joint_rocker_to_bogie_right',

            # Wheel joints
            'wheel_front_left',
            'wheel_middle_left',
            'wheel_back_left',
            'wheel_front_right',
            'wheel_middle_right',
            'wheel_back_right'
        ]

        # Initialize joint positions (all start at 0.0)
        self.joint_positions = [0.0] * len(self.joint_names)

    def wrap_angle(self, angle):
        """Wrap the angle between 0 and 2*pi."""
        return angle % (2 * math.pi)

    def publish_joint_states(self):
        """Continuously publish joint states."""
        while not rospy.is_shutdown():
            # Create and populate the JointState message
            joint_state_msg = JointState()
            joint_state_msg.header = Header(stamp=rospy.Time.now())
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = self.joint_positions

            # Simulate revolute joint movement with angle wrapping
            self.joint_positions[3] = self.wrap_angle(self.joint_positions[3] + 0.01)  # joint_cylinder_left_to_rocker_left
            self.joint_positions[4] = self.wrap_angle(self.joint_positions[4] + 0.01)  # joint_cylinder_left_to_rocker_right
            self.joint_positions[5] = self.wrap_angle(self.joint_positions[5] + 0.01)  # joint_rocker_to_bogie_left
            self.joint_positions[6] = self.wrap_angle(self.joint_positions[6] + 0.01)  # joint_rocker_to_bogie_right

            # Simulate wheel joint rotation with wrapping
            for i in range(7, 13):  # Loop over all wheel joints
                self.joint_positions[i] = self.wrap_angle(self.joint_positions[i] + 0.1)

            # Publish the joint states
            self.joint_pub.publish(joint_state_msg)

            # Wait for the next cycle
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = JointStatePublisher()
        publisher.publish_joint_states()
    except rospy.ROSInterruptException:
        pass
