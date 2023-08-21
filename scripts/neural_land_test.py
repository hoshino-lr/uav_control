#!/usr/bin/env python3
import random
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf_conversions
import rospy
import time
import math


class NeuralTest(object):

    def __init__(self):
        # Set point publishing MUST be faster than 2Hz

        self.sub_state_topic = "mavros/state"
        self.pub_pose_topic = "uav/command/pose"

        self.srv_mode_topic = "/mavros/set_mode"
        self.sub_pose_topic = "/vrpn_client_node/uav_nx/pose"

        self.rate = rospy.Rate(50)

        self.current_state = State()
        self.set_pose = PoseStamped()

        self.motion_pose = PoseStamped()

        # create parameter generator

        self.sub_pose = rospy.Subscriber(self.sub_pose_topic, PoseStamped, callback=self.callback_pose,
                                         tcp_nodelay=True)
        self.sub_state = rospy.Subscriber(self.sub_state_topic, State, callback=self.callback_state, tcp_nodelay=True)
        self.pub_pose = rospy.Publisher(self.pub_pose_topic, PoseStamped, queue_size=1, tcp_nodelay=True)
        self.set_mode_client = rospy.ServiceProxy(self.srv_mode_topic, SetMode)

        self.safe_x_max, self.safe_x_min, self.safe_y_max, self.safe_y_min, self.safe_z_max, self.safe_z_min = \
            1, -1, 2, -2, 1.5, 0

        time.sleep(1)

        """-----------------------------------------------------------"""
        self.radius = 0.5  # m
        self.land_z = 0.8  # m
        self.takeoff_z = 1.4  # m
        self.point_x, self.point_y = -0.6, 0.5
        self.valid_times_max = 150
        self.valid_times = 0
        self.valid_radius = 0.2
        """-----------------------------------------------------------"""
        self.count = 0
        self.count_max = 20
        """-----------------------------------------------------------"""
        self.now_point = self.generate_point(self.land_z)
        self.set_pose.pose.position.x, self.set_pose.pose.position.y, \
            self.set_pose.pose.position.z = self.now_point[0], self.now_point[1], self.now_point[2]
        self.set_pose.pose.orientation.w = 1

    def distance(self, input_point):
        return math.dist(input_point, [self.motion_pose.pose.position.x,
                                       self.motion_pose.pose.position.y,
                                       self.motion_pose.pose.position.z])

    def check_point(self):
        if self.valid_times < self.valid_times_max:
            if self.distance(self.now_point) < self.valid_radius:
                self.valid_times += 1
        else:
            self.valid_times = 0
            self.count += 1
            self.now_point = self.generate_point()
            self.set_pose.pose.position.x, self.set_pose.pose.position.y, \
                self.set_pose.pose.position.z = self.now_point[0], self.now_point[1], self.now_point[2]

    def generate_point(self, z=0.):
        x = random.uniform(- self.radius / 2, self.radius / 2) + self.point_x
        y = random.uniform(- self.radius / 2, self.radius / 2) + self.point_y
        if z == 0:
            if self.now_point[2] == self.land_z:
                z = self.takeoff_z
            else:
                z = self.land_z
        return [x, y, z]

    def land_mode(self):
        # land
        while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
            self.set_mode_client(0, "AUTO.LAND")
            self.rate.sleep()
        rospy.loginfo("LAND success")

    def spin(self):
        while not rospy.is_shutdown() and self.count < self.count_max:
            if not self.check_position():
                self.land_mode()
                break
            self.check_point()
            self.pub_pose.publish(self.set_pose)
            self.rate.sleep()
        self.land_mode()

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


if __name__ == "__main__":
    rospy.init_node("neural_test")
    test = NeuralTest()
    test.spin()
