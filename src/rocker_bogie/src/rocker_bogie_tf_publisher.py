#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros

def rocker_bogie_tf_publisher():
    rospy.init_node('rocker_bogie_tf_publisher')

    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Transform from odom to base_link
        # odom_to_base = TransformStamped()
        # odom_to_base.header.stamp = rospy.Time.now()
        # odom_to_base.header.frame_id = "odom"
        # odom_to_base.child_frame_id = "base_link"
        # odom_to_base.transform.translation.x = 0.0
        # odom_to_base.transform.translation.y = 0.0
        # odom_to_base.transform.translation.z = 0.0
        # odom_to_base.transform.rotation.x = 0.0
        # odom_to_base.transform.rotation.y = 0.0
        # odom_to_base.transform.rotation.z = 0.0
        # odom_to_base.transform.rotation.w = 1.0
        # broadcaster.sendTransform(odom_to_base)

        base_to_footprint = TransformStamped()
        base_to_footprint.header.stamp = rospy.Time.now()
        base_to_footprint.header.frame_id = "base_link"
        base_to_footprint.child_frame_id = "base_footprint"
        base_to_footprint.transform.translation.x = 0.0
        base_to_footprint.transform.translation.y = 0.0
        base_to_footprint.transform.translation.z = -0.05  # Adjust based on the robot's geometry
        base_to_footprint.transform.rotation.x = 0.0
        base_to_footprint.transform.rotation.y = 0.0
        base_to_footprint.transform.rotation.z = 0.0
        base_to_footprint.transform.rotation.w = 1.0

        broadcaster.sendTransform(base_to_footprint)

        # Transform from base_link to lidar_link
        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = rospy.Time.now()
        base_to_lidar.header.frame_id = "base_link"
        base_to_lidar.child_frame_id = "lidar_link"
        base_to_lidar.transform.translation.x = 0.0
        base_to_lidar.transform.translation.y = 0.0
        base_to_lidar.transform.translation.z = 0.2
        base_to_lidar.transform.rotation.x = 0.0
        base_to_lidar.transform.rotation.y = 0.0
        base_to_lidar.transform.rotation.z = 0.0
        base_to_lidar.transform.rotation.w = 1.0
        broadcaster.sendTransform(base_to_lidar)

        # Transform from base_link to rocker_left
        base_to_rocker_left = TransformStamped()
        base_to_rocker_left.header.stamp = rospy.Time.now()
        base_to_rocker_left.header.frame_id = "base_link"
        base_to_rocker_left.child_frame_id = "rocker_left"
        base_to_rocker_left.transform.translation.x = 0.0
        base_to_rocker_left.transform.translation.y = 0.145
        base_to_rocker_left.transform.translation.z = 0.0
        base_to_rocker_left.transform.rotation.x = 0.0
        base_to_rocker_left.transform.rotation.y = 0.0
        base_to_rocker_left.transform.rotation.z = 0.0
        base_to_rocker_left.transform.rotation.w = 1.0
        broadcaster.sendTransform(base_to_rocker_left)

        # Transform from base_link to rocker_right
        base_to_rocker_right = TransformStamped()
        base_to_rocker_right.header.stamp = rospy.Time.now()
        base_to_rocker_right.header.frame_id = "base_link"
        base_to_rocker_right.child_frame_id = "rocker_right"
        base_to_rocker_right.transform.translation.x = 0.0
        base_to_rocker_right.transform.translation.y = -0.145
        base_to_rocker_right.transform.translation.z = 0.0
        base_to_rocker_right.transform.rotation.x = 0.0
        base_to_rocker_right.transform.rotation.y = 0.0
        base_to_rocker_right.transform.rotation.z = 0.0
        base_to_rocker_right.transform.rotation.w = 1.0
        broadcaster.sendTransform(base_to_rocker_right)

        # Transform from rocker_left to bogie_left
        rocker_left_to_bogie_left = TransformStamped()
        rocker_left_to_bogie_left.header.stamp = rospy.Time.now()
        rocker_left_to_bogie_left.header.frame_id = "rocker_left"
        rocker_left_to_bogie_left.child_frame_id = "bogie_left"
        rocker_left_to_bogie_left.transform.translation.x = 0.1
        rocker_left_to_bogie_left.transform.translation.y = 0.0
        rocker_left_to_bogie_left.transform.translation.z = -0.1
        rocker_left_to_bogie_left.transform.rotation.x = 0.0
        rocker_left_to_bogie_left.transform.rotation.y = 0.0
        rocker_left_to_bogie_left.transform.rotation.z = 0.0
        rocker_left_to_bogie_left.transform.rotation.w = 1.0
        broadcaster.sendTransform(rocker_left_to_bogie_left)

        # Transform from rocker_right to bogie_right
        rocker_right_to_bogie_right = TransformStamped()
        rocker_right_to_bogie_right.header.stamp = rospy.Time.now()
        rocker_right_to_bogie_right.header.frame_id = "rocker_right"
        rocker_right_to_bogie_right.child_frame_id = "bogie_right"
        rocker_right_to_bogie_right.transform.translation.x = 0.1
        rocker_right_to_bogie_right.transform.translation.y = 0.0
        rocker_right_to_bogie_right.transform.translation.z = -0.1
        rocker_right_to_bogie_right.transform.rotation.x = 0.0
        rocker_right_to_bogie_right.transform.rotation.y = 0.0
        rocker_right_to_bogie_right.transform.rotation.z = 0.0
        rocker_right_to_bogie_right.transform.rotation.w = 1.0
        broadcaster.sendTransform(rocker_right_to_bogie_right)

        # Transform for all wheels (example: front_left, repeat for others)
        bogie_left_to_front_left_wheel = TransformStamped()
        bogie_left_to_front_left_wheel.header.stamp = rospy.Time.now()
        bogie_left_to_front_left_wheel.header.frame_id = "bogie_left"
        bogie_left_to_front_left_wheel.child_frame_id = "wheel_front_left"
        bogie_left_to_front_left_wheel.transform.translation.x = 0.2
        bogie_left_to_front_left_wheel.transform.translation.y = 0.03
        bogie_left_to_front_left_wheel.transform.translation.z = -0.07
        bogie_left_to_front_left_wheel.transform.rotation.x = 0.0
        bogie_left_to_front_left_wheel.transform.rotation.y = 0.0
        bogie_left_to_front_left_wheel.transform.rotation.z = 0.0
        bogie_left_to_front_left_wheel.transform.rotation.w = 1.0
        broadcaster.sendTransform(bogie_left_to_front_left_wheel)

        # Repeat similar blocks for all wheels
        # Example for wheel_middle_left
        bogie_left_to_middle_left_wheel = TransformStamped()
        bogie_left_to_middle_left_wheel.header.stamp = rospy.Time.now()
        bogie_left_to_middle_left_wheel.header.frame_id = "bogie_left"
        bogie_left_to_middle_left_wheel.child_frame_id = "wheel_middle_left"
        bogie_left_to_middle_left_wheel.transform.translation.x = -0.12
        bogie_left_to_middle_left_wheel.transform.translation.y = 0.03
        bogie_left_to_middle_left_wheel.transform.translation.z = -0.07
        bogie_left_to_middle_left_wheel.transform.rotation.x = 0.0
        bogie_left_to_middle_left_wheel.transform.rotation.y = 0.0
        bogie_left_to_middle_left_wheel.transform.rotation.z = 0.0
        bogie_left_to_middle_left_wheel.transform.rotation.w = 1.0
        broadcaster.sendTransform(bogie_left_to_middle_left_wheel)

        rospy.loginfo("Publishing transforms for rocker-bogie.")
        rate.sleep()

if __name__ == "__main__":
    try:
        rocker_bogie_tf_publisher()
    except rospy.ROSInterruptException:
        pass
