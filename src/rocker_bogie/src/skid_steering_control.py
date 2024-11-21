#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class SkidSteeringControl:
    def __init__(self):
        rospy.init_node('skid_steering_control')

        # Publishers for left and right wheel velocity commands
        self.vel_left_front_pub = rospy.Publisher('/cmd_vel_left_front', Twist, queue_size=10)
        self.vel_left_middle_pub = rospy.Publisher('/cmd_vel_left_middle', Twist, queue_size=10)
        self.vel_left_back_pub = rospy.Publisher('/cmd_vel_left_back', Twist, queue_size=10)
        self.vel_right_front_pub = rospy.Publisher('/cmd_vel_right_front', Twist, queue_size=10)
        self.vel_right_middle_pub = rospy.Publisher('/cmd_vel_right_middle', Twist, queue_size=10)
        self.vel_right_back_pub = rospy.Publisher('/cmd_vel_right_back', Twist, queue_size=10)

        self.avg_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize velocity messages for each wheel
        self.left_front_velocity = Twist()
        self.left_middle_velocity = Twist()
        self.left_back_velocity = Twist()
        self.right_front_velocity = Twist()
        self.right_middle_velocity = Twist()
        self.right_back_velocity = Twist()

        self.avg_velocity = Twist()

        # Set up keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        rospy.loginfo("Use WASD keys to control the rover. Press 'q' to quit.")

    def publish_velocity(self):
        """Publish the set velocities to each wheel."""
        self.vel_left_front_pub.publish(self.left_front_velocity)
        self.vel_left_middle_pub.publish(self.left_middle_velocity)
        self.vel_left_back_pub.publish(self.left_back_velocity)
        self.vel_right_front_pub.publish(self.right_front_velocity)
        self.vel_right_middle_pub.publish(self.right_middle_velocity)
        self.vel_right_back_pub.publish(self.right_back_velocity)

        self.avg_vel_pub.publish(self.avg_velocity)

    def set_velocity(self, vel):
        """Set velocity for each individual wheel."""
        self.left_front_velocity.angular.z = vel[0]
        self.left_middle_velocity.angular.z = vel[1]
        self.left_back_velocity.angular.z = vel[2]
        self.right_front_velocity.angular.z = vel[3]
        self.right_middle_velocity.angular.z = vel[4]
        self.right_back_velocity.angular.z = vel[5]
    
    def scalar_multiply(self, scalar, list_values):
        return [x * scalar for x in list_values]

    def avg(self, value):
        return sum(value)/len(value)


    def on_press(self, key):
        """Handle key press events."""

        forward = [50, 50, 50, 50, 50, 50]
        backward = [-50, -50, -50, -50, -50, -50]
        # left_turn = [-40, -35, -30, 30, 35, 40]
        # right_turn = [30, 35, 40, -40, -35, -30]
        left_turn = [-50, -50, -50, 50, 50, 50]
        right_turn = [50, 50, 50, -50, -50, -50]

        scalar = 0.5
        forward_scaled = self.scalar_multiply(scalar, forward)
        backward_scaled = self.scalar_multiply(scalar, backward)
        left_turn_scaled = self.scalar_multiply(scalar, left_turn)
        right_turn_scaled = self.scalar_multiply(scalar, right_turn)


        try:
            if key.char == 'w':  # Forward
                self.set_velocity(forward_scaled)
                self.avg_velocity.linear.x = self.avg(forward_scaled)
            elif key.char == 's':  # Backward
                self.set_velocity(backward_scaled)
                self.avg_velocity.linear.x = self.avg(backward_scaled)
            elif key.char == 'a':  # Left turn
                self.set_velocity(left_turn_scaled)
                self.avg_velocity.angular.z = 1#self.avg(left_turn_scaled)
            elif key.char == 'd':  # Right turn
                self.set_velocity(right_turn_scaled)
                self.avg_velocity.angular.z = -1#self.avg(right_turn_scaled)
            elif key.char == 'q':  # Quit
                rospy.signal_shutdown("User requested shutdown.")
           
            # Publish the velocities
            self.publish_velocity()

        except AttributeError:
            # Handle special keys (if any)
            pass

    def on_release(self, key):
        """Handle key release events."""
        # Stop the rover when any key is released
        self.set_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.avg_velocity.linear.x = 0
        self.avg_velocity.angular.z = 0
        self.publish_velocity()

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
