#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class SkidSteeringControl:
    def __init__(self):
        rospy.init_node('skid_steering_control')

        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a Twist message to store velocity
        self.twist = Twist()

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def on_press(self, key):
        """Handle key press events."""
        try:
            if key.char == 'w':
                self.twist.linear.x = 1.5  # Move forward
                self.twist.angular.z = 0.0
            elif key.char == 's':
                self.twist.linear.x = -5.0  # Move backward
                self.twist.angular.z = 0.0
            elif key.char == 'a':
                self.twist.angular.z = 5.0  # Rotate left (skid-steer)
                self.twist.linear.x = 0.0
            elif key.char == 'd':
                self.twist.angular.z = -5.0  # Rotate right (skid-steer)
                self.twist.linear.x = 0.0
            elif key.char == 'q':
                rospy.signal_shutdown("User requested shutdown.")  # Exit gracefully

            # Publish the Twist message
            self.cmd_vel_pub.publish(self.twist)

        except AttributeError:
            # Handle special keys (if any)
            pass

    def on_release(self, key):
        """Handle key release events."""
        # Stop the rover when any key is released
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

        # Stop the listener if 'q' is pressed
        if key == keyboard.Key.esc:
            return False  # Stop the listener

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = SkidSteeringControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
