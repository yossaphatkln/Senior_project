#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pynput import keyboard

class SkidSteeringControl:
    def __init__(self):
        rospy.init_node('skid_steering_control')

        # Publishers for left and right wheel velocity commands
        self.cmd_vel_left_pub = rospy.Publisher('/cmd_vel_left', Float64, queue_size=10)
        self.cmd_vel_right_pub = rospy.Publisher('/cmd_vel_right', Float64, queue_size=10)

        # Initialize left and right velocities
        self.left_velocity = Float64()
        self.right_velocity = Float64()

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def on_press(self, key):
        """Handle key press events."""
        try:
            # Forward and backward control
            if key.char == 'w':
                self.left_velocity.data = 20
                self.right_velocity.data = 20
            elif key.char == 's':
                self.left_velocity.data = -20
                self.right_velocity.data = -20
            elif key.char == 'a':
                self.left_velocity.data = -20
                self.right_velocity.data = 20
            elif key.char == 'd':
                self.left_velocity.data = 20
                self.right_velocity.data = -20

            elif key.char == 'q':
                rospy.signal_shutdown("User requested shutdown.")  # Exit gracefully

            # Publish the velocities
            self.cmd_vel_left_pub.publish(self.left_velocity)
            self.cmd_vel_right_pub.publish(self.right_velocity)

        except AttributeError:
            # Handle special keys (if any)
            pass

    def on_release(self, key):
        """Handle key release events."""
        # Stop the rover when any key is released
        self.left_velocity.data = 0.0
        self.right_velocity.data = 0.0
        self.cmd_vel_left_pub.publish(self.left_velocity)
        self.cmd_vel_right_pub.publish(self.right_velocity)

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
