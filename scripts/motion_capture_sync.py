#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# 接收到位置、速度信息再转发出去

import geometry_msgs.msg
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
import tf_conversions

pub_pose_topic = '/mavros/vision_pose/pose'
pub_odom_topic = '/px4/vision_odom'
sub_pose_topic = "/vrpn_client_node/uav_nx/pose"
sub_twist_topic = "/vrpn_client_node/uav_nx/twist"

use_fake_position = rospy.get_param('use_fake_position', False)
# init node
rospy.init_node('motion_capture_sync')
motion_data = geometry_msgs.msg.PoseStamped()
motion_speed = Vector3Stamped()
motion_odom = Odometry()
pub_pose = rospy.Publisher(pub_pose_topic, PoseStamped, queue_size=1, tcp_nodelay=True)
pub_odom = rospy.Publisher(pub_odom_topic, Odometry, queue_size=1, tcp_nodelay=True)
# init odom
motion_odom.child_frame_id = "base_link"
motion_odom.header.frame_id = "odom"
motion_odom.pose.covariance = [1, 0, 0, 0, 0, 0,
                               0, 1, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0,
                               0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1]

motion_odom.twist.covariance = [-1, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0]


def callback_all(pose_data: PoseStamped, twist_data: TwistStamped):
    global motion_data, pub_pose, motion_odom, pub_odom
    motion_data.pose.position.x = pose_data.pose.position.x / 1000
    motion_data.pose.position.y = pose_data.pose.position.y / 1000
    motion_data.pose.position.z = pose_data.pose.position.z / 1000
    motion_data.pose.orientation = pose_data.pose.orientation
    motion_data.header.frame_id = pose_data.header.frame_id
    motion_data.header.stamp = pose_data.header.stamp

    motion_odom.header.stamp = pose_data.header.stamp
    motion_odom.pose.pose = motion_data.pose
    motion_odom.twist.twist.angular = twist_data.twist.angular
    motion_odom.twist.twist.linear.x = twist_data.twist.linear.x / 1000
    motion_odom.twist.twist.linear.y = twist_data.twist.linear.y / 1000
    motion_odom.twist.twist.linear.z = twist_data.twist.linear.z / 1000

    pub_pose.publish(motion_data)


pose_sub = message_filters.Subscriber(sub_pose_topic, PoseStamped)
twist_sub = message_filters.Subscriber(sub_twist_topic, TwistStamped)
all_sub = message_filters.ApproximateTimeSynchronizer([pose_sub, twist_sub], 1, 1, allow_headerless=True)
all_sub.registerCallback(callback_all)

r = rospy.Rate(50)  # limited by bit rate of radio telemetry

while not rospy.is_shutdown():
    if use_fake_position:
        motion_data.pose.position.x = 0
        motion_data.pose.position.y = 0
        motion_data.pose.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        motion_data.pose.orientation.x = q[0]
        motion_data.pose.orientation.y = q[1]
        motion_data.pose.orientation.z = q[2]
        motion_data.pose.orientation.w = q[3]
        motion_data.header.stamp = rospy.get_rostime()
        pub_pose.publish(motion_data)
        r.sleep()
    else:
        pub_odom.publish(motion_odom)
        r.sleep()
        # rospy.spin()  # equal to a while loop
