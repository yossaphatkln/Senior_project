#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios

def get_key():
    """ Get a single keypress from the terminal. """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rospy.init_node('rocker_bogie_control')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    
    print("Control the rover!")
    print("Press 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right.")
    print("Press 'q' to quit.")

    while not rospy.is_shutdown():
        key = get_key()

        if key == 'w':
            twist.linear.x = 1.0  # Move forward
            twist.angular.z = 0.0
        elif key == 's':
            twist.linear.x = -1.0  # Move backward
            twist.angular.z = 0.0
        elif key == 'a':
            twist.linear.x = 0.0
            twist.angular.z = 1.0  # Turn left
        elif key == 'd':
            twist.linear.x = 0.0
            twist.angular.z = -1.0  # Turn right
        elif key == 'q':
            break  # Quit
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        pub.publish(twist)
        rate.sleep()

    # Stop the rover before exiting
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
