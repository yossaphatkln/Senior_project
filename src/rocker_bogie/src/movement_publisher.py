#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf
import math

# Wheel base distance (distance between left and right wheels)
WHEEL_BASE = 0.4 * 12.5

# Initialize velocities for each wheel
left_front_vel = 0.0
left_middle_vel = 0.0
left_back_vel = 0.0
right_front_vel = 0.0
right_middle_vel = 0.0
right_back_vel = 0.0

# Callback functions to update wheel velocities
def left_front_callback(msg):
    global left_front_vel
    left_front_vel = msg.angular.z

def left_middle_callback(msg):
    global left_middle_vel
    left_middle_vel = msg.angular.z

def left_back_callback(msg):
    global left_back_vel
    left_back_vel = msg.angular.z

def right_front_callback(msg):
    global right_front_vel
    right_front_vel = msg.angular.z

def right_middle_callback(msg):
    global right_middle_vel
    right_middle_vel = msg.angular.z

def right_back_callback(msg):
    global right_back_vel
    right_back_vel = msg.angular.z

def dummy_odometry_publisher():
    rospy.init_node('dummy_odometry_publisher')
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    # Subscribe to each wheel's velocity topic
    rospy.Subscriber("/cmd_vel_left_front", Twist, left_front_callback)
    rospy.Subscriber("/cmd_vel_left_middle", Twist, left_middle_callback)
    rospy.Subscriber("/cmd_vel_left_back", Twist, left_back_callback)
    rospy.Subscriber("/cmd_vel_right_front", Twist, right_front_callback)
    rospy.Subscriber("/cmd_vel_right_middle", Twist, right_middle_callback)
    rospy.Subscriber("/cmd_vel_right_back", Twist, right_back_callback)

    # Initial position and orientation
    x = 0.0
    y = 0.0
    theta = 0.0

    rate = rospy.Rate(10)  # 10 Hz
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        last_time = current_time

        # Calculate average velocities for left and right wheel sets
        left_avg = (left_front_vel + left_middle_vel + left_back_vel) / 3/20
        right_avg = (right_front_vel + right_middle_vel + right_back_vel) / 3/20

        # Calculate linear and angular velocities
        vx = (left_avg + right_avg) / 2.0  # Linear velocity
        vth = (right_avg - left_avg) / WHEEL_BASE  # Angular velocity

        # Determine movement direction based on wheel velocities
        moving_forward = left_avg > 0 and right_avg > 0
        moving_backward = left_avg < 0 and right_avg < 0
        rotating_left = left_avg < 0 and right_avg > 0
        rotating_right = left_avg > 0 and right_avg < 0

        # Update dx, dy, and dth based on movement type
        if moving_forward or moving_backward:
            # Moving forward or backward
            dx = vx * math.cos(theta) * dt
            dy = vx * math.sin(theta) * dt
            dth = 0
        elif rotating_left or rotating_right:
            # Rotating left or right
            dx = 0
            dy = 0
            dth = vth * dt
        else:
            dx = dy = dth = 0  # In case of no movement


        # Update odometry
        x += dx
        y += dy
        theta += dth

        # Create a quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        # Publish the transform over TF
        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        # Set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        # Publish the odometry message
        odom_pub.publish(odom)

        rate.sleep()

if __name__ == '__main__':
    try:
        dummy_odometry_publisher()
    except rospy.ROSInterruptException:
        pass
