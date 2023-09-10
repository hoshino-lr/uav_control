#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandLong, CommandLongRequest
import tf_conversions
import rospy
import time
import math


class LandInterface(object):

    def __init__(self, land_point, pub_topic="uav/command/pose"):
        # Set point publishing MUST be faster than 2Hz

        self.sub_state_topic = "mavros/state"
        self.pub_pose_topic = pub_topic
        self.srv_arm_topic = "/mavros/cmd/arming"
        self.srv_command_topic = "/mavros/cmd/command"
        self.srv_mode_topic = "/mavros/set_mode"
        self.sub_pose_topic = "/vrpn_client_node/uav_nx/pose"

        self.rate = rospy.Rate(50)

        self.current_state = State()
        self.set_pose = PoseStamped()

        self.motion_pose = PoseStamped()
        self.point_lists = []
        # create parameter generator
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = False

        self.long_cmd = CommandLongRequest()

        self.sub_pose = rospy.Subscriber(self.sub_pose_topic, PoseStamped, callback=self.callback_pose,
                                         tcp_nodelay=True)
        self.sub_state = rospy.Subscriber(self.sub_state_topic, State, callback=self.callback_state, tcp_nodelay=True)
        self.pub_pose = rospy.Publisher(self.pub_pose_topic, PoseStamped, queue_size=1, tcp_nodelay=True)
        self.set_mode_client = rospy.ServiceProxy(self.srv_mode_topic, SetMode)
        self.arming_client = rospy.ServiceProxy(self.srv_arm_topic, CommandBool)
        self.force_arm_client = rospy.ServiceProxy(self.srv_command_topic, CommandLong)
        self.safe_x_max, self.safe_x_min, self.safe_y_max, self.safe_y_min, self.safe_z_max, self.safe_z_min = \
            2.5, -1, 1.5, -1.5, 1.7, 0

        self.set_pose.pose.orientation.w = 1

    def distance(self, input_point):
        return math.dist(input_point, [self.motion_pose.pose.position.x,
                                       self.motion_pose.pose.position.y,
                                       self.motion_pose.pose.position.z])


    def land_mode(self):
        # land
        while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
            self.set_mode_client(0, "AUTO.LAND")
            self.rate.sleep()
        rospy.loginfo("LAND success")

    def disarm(self):
        # disarm
        while not rospy.is_shutdown():
            if self.arming_client.call(self.arm_cmd).success == True:
                break
            self.rate.sleep()
        rospy.loginfo("DISARM success")

    def force_disarm(self):
        # force disarm
        self.long_cmd.command = 400
        self.long_cmd.param1 = 0
        self.long_cmd.param2 = 21196
        while not rospy.is_shutdown():
            if self.force_arm_client.call(self.long_cmd).success == True:
                break
            self.rate.sleep()
            rospy.loginfo("DISARM FAILED")
        rospy.loginfo("DISARM success")

    def actuator_output_set(self, t):
        # force disarm
        self.long_cmd.command = 187
        self.long_cmd.param1 = math.cos(t)
        self.set_pose.pose.position.x = self.long_cmd.param1 * 500 + 1500
        self.long_cmd.param2 = -1
        self.long_cmd.param3 = -1
        self.long_cmd.param4 = -1
        self.pub_pose.publish(self.set_pose)
        while not rospy.is_shutdown():
            if self.force_arm_client.call(self.long_cmd).success == True:
                break
            self.rate.sleep()
            rospy.loginfo("SET failed")
        rospy.loginfo("SET success")

    def check_position(self) -> bool:
        if self.safe_x_max > self.motion_pose.pose.position.x > self.safe_x_min and \
                self.safe_y_max > self.motion_pose.pose.position.y > self.safe_y_min and \
                self.safe_z_max > self.motion_pose.pose.position.z:
            return True
        else:
            return False

    def callback_state(self, msg):
        self.current_state = msg

    def callback_pose(self, pose_data: PoseStamped):
        self.motion_pose.pose.position.x = pose_data.pose.position.x / 1000
        self.motion_pose.pose.position.y = pose_data.pose.position.y / 1000
        self.motion_pose.pose.position.z = pose_data.pose.position.z / 1000
        self.motion_pose.pose.orientation = pose_data.pose.orientation
