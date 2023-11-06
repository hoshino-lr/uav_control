#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, CommandLongRequest, CommandLong
import rospy
import time


class NeuralTest(object):

    def __init__(self):
        # Set point publishing MUST be faster than 2Hz

        self.sub_state_topic = "mavros/state"
        self.srv_command_topic = "/mavros/cmd/command"
        self.srv_mode_topic = "/mavros/set_mode"
        self.sub_pose_topic = '/px4/vision_odom'

        self.rate = rospy.Rate(50)

        self.current_state = State()
        self.set_pose = PoseStamped()

        self.motion_pose = PoseStamped()
        self.long_cmd = CommandLongRequest()
        # create parameter generator
        self.force_arm_client = rospy.ServiceProxy(self.srv_command_topic, CommandLong)
        self.sub_pose = rospy.Subscriber(self.sub_pose_topic, Odometry, callback=self.callback_pose,
                                         tcp_nodelay=True)
        self.sub_state = rospy.Subscriber(self.sub_state_topic, State, callback=self.callback_state, tcp_nodelay=True)
        self.set_mode_client = rospy.ServiceProxy(self.srv_mode_topic, SetMode)

        self.safe_x_max, self.safe_x_min, self.safe_y_max, self.safe_y_min, self.safe_z_max, self.safe_z_min = \
            1.2, -1.2, 2.2, -0.5, 1.5, 0

        time.sleep(1)

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

    def land_mode(self):
        # land
        while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
            self.set_mode_client(0, "AUTO.LAND")
            self.rate.sleep()
        rospy.loginfo("LAND success")

    def spin(self):
        while not rospy.is_shutdown():
            if not self.check_position():
                self.land_mode()
                # self.force_disarm()
                break
            self.rate.sleep()

    def check_position(self) -> bool:
        if self.safe_x_max > self.motion_pose.pose.position.x > self.safe_x_min and \
                self.safe_y_max > self.motion_pose.pose.position.y > self.safe_y_min and \
                self.safe_z_max > self.motion_pose.pose.position.z:
            return True
        else:
            return False

    def callback_state(self, msg):
        self.current_state = msg

    def callback_pose(self, pose_data: Odometry):
        self.motion_pose.pose.position.x = pose_data.pose.pose.position.x
        self.motion_pose.pose.position.y = pose_data.pose.pose.position.y
        self.motion_pose.pose.position.z = pose_data.pose.pose.position.z


if __name__ == "__main__":
    rospy.init_node("safe_area")
    test = NeuralTest()
    test.spin()
