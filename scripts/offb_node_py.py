#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import math
import rospy
import threading
import time
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandLong, CommandLongRequest
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf_conversions
from dynamic_reconfigure.server import Server
from uav_control.cfg import uavConfig
from simple_pid import PID

class UAVController(object):

    def __init__(self):
        # Set point publishing MUST be faster than 2Hz

        self.sub_state_topic = "mavros/state"
        self.pub_pose_topic = "mavros/setpoint_position/local"
        self.pub_velocity_topic = "/mavros/setpoint_velocity/cmd_vel"
        self.pub_attitude_topic = "mavros/setpoint_raw/attitude"
        self.srv_arm_topic = "/mavros/cmd/arming"
        self.srv_mode_topic = "/mavros/set_mode"
        self.sub_pose_topic = "/vrpn_client_node/uav_nx/pose"
        self.srv_command_topic = "/mavros/cmd/command"

        self.rate = rospy.Rate(50)
        self.current_state = State()

        self.long_cmd = CommandLongRequest()
        self.set_state = State()
        self.set_pose = PoseStamped()
        self.set_velocity = TwistStamped()
        self.set_attitude = AttitudeTarget()
        self.set_attitude.type_mask = 3
        self.motion_pose = PoseStamped()

        self.quit = False
        self.takeoff = False
        self.attitude_mode = False

        # create parameter generator
        self.srv_dyn = Server(uavConfig, self.callback_dyn)
        self.sub_pose = rospy.Subscriber(self.sub_pose_topic, PoseStamped, callback=self.callback_pose,
                                         tcp_nodelay=True)
        self.sub_state = rospy.Subscriber(self.sub_state_topic, State, callback=self.callback_state, tcp_nodelay=True)
        self.pub_pose = rospy.Publisher(self.pub_pose_topic, PoseStamped, queue_size=1, tcp_nodelay=True)
        self.pub_velocity = rospy.Publisher(self.pub_velocity_topic, TwistStamped, queue_size=1, tcp_nodelay=True)
        self.pub_attitude = rospy.Publisher(self.pub_attitude_topic, AttitudeTarget, queue_size=1, tcp_nodelay=True)
        rospy.wait_for_service(self.srv_arm_topic)
        self.arming_client = rospy.ServiceProxy(self.srv_arm_topic, CommandBool)
        rospy.wait_for_service(self.srv_mode_topic)
        self.set_mode_client = rospy.ServiceProxy(self.srv_mode_topic, SetMode)
        self.srv_command = self.force_arm_client = rospy.ServiceProxy(self.srv_command_topic, CommandLong)
        # param
        self.roll, self.pitch, self.yaw = 0, 0, 0
        self.safe_x_max, self.safe_x_min, self.safe_y_max, self.safe_y_min, self.safe_z_max, self.safe_z_min = \
            0, 0, 0, 0, 0, 0
        self.mode = 0
        self.start_listen_cli()

        time.sleep(1)
        self.pose_init()

    def pose_init(self):
        self.set_pose.pose.position.x = self.motion_pose.pose.position.x
        self.set_pose.pose.position.y = self.motion_pose.pose.position.y
        self.set_pose.pose.position.z = self.motion_pose.pose.position.z
        self.srv_dyn.update_configuration({"position_x": self.set_pose.pose.position.x,
                                           "position_y": self.set_pose.pose.position.y,
                                           "position_z": self.set_pose.pose.position.z})
        self.srv_dyn.update_configuration({"mode": 0, "QUIT": False, "takeoff": False})

    def start_listen_cli(self):
        thread = threading.Thread(target=self.ros_spin)
        thread.daemon = True
        thread.start()

    @staticmethod
    def ros_spin():
        rospy.spin()

    def land_mode(self):
        # land
        while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
            self.set_mode_client(0, "AUTO.LAND")
            self.rate.sleep()
        rospy.loginfo("LAND success")

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

    def spin(self):
        while not rospy.is_shutdown() and not self.quit:
            if self.takeoff:
                self.takeoff_mode()
                self.takeoff = False
                self.srv_dyn.update_configuration({"takeoff": False})
            if self.attitude_mode:
                self.callback_attitudemode()
                self.attitude_mode = False
                self.srv_dyn.update_configuration({"attitude_mode": False})
            if self.mode == 0:
                if not self.check_position():
                    self.force_disarm()
                    break
            elif self.mode == 1:
                if not self.check_position():
                    self.force_disarm()
                    break
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
                self.set_pose.pose.orientation.x = q[0]
                self.set_pose.pose.orientation.y = q[1]
                self.set_pose.pose.orientation.z = q[2]
                self.set_pose.pose.orientation.w = q[3]
                self.pub_pose.publish(self.set_pose)
            elif self.mode == 2:
                if not self.check_position():
                    self.force_disarm()
                    break
                q = tf_conversions.transformations.quaternion_from_euler(self.roll, self.pitch, 0)
                self.set_attitude.orientation.x = q[0]
                self.set_attitude.orientation.y = q[1]
                self.set_attitude.orientation.z = q[2]
                self.set_attitude.orientation.w = q[3]
                self.pub_attitude.publish(self.set_attitude)
            elif self.mode == 3:
                if not self.check_position():
                    self.force_disarm()
                    break
                self.pub_velocity.publish(self.set_velocity)
            elif self.mode == 4:
                self.force_disarm()
            self.rate.sleep()

    def takeoff_mode(self):
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        while not rospy.is_shutdown() and self.takeoff:
            if self.current_state.mode != "OFFBOARD":
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")
            else:
                if not self.current_state.armed and self.takeoff:
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")
                        break
                if self.current_state.armed:
                    break
            self.pub_pose.publish(self.set_pose)
            self.rate.sleep()

    def check_position(self) -> bool:
        if self.safe_x_max > self.motion_pose.pose.position.x > self.safe_x_min and \
                self.safe_y_max > self.motion_pose.pose.position.y > self.safe_y_min and \
                self.safe_z_max > self.motion_pose.pose.position.z:
            return True
        else:
            return False

    def callback_pose(self, pose_data: PoseStamped):
        self.motion_pose.pose.position.x = pose_data.pose.position.x / 1000
        self.motion_pose.pose.position.y = pose_data.pose.position.y / 1000
        self.motion_pose.pose.position.z = pose_data.pose.position.z / 1000
        self.motion_pose.pose.orientation = pose_data.pose.orientation

    def callback_altitudemode(self):
        pass

    def fixed_height(self, target_height):
        # 位置环
        position_loop_pid = PID(2.5, 1, 0.1, setpoint=target_height)
        position_loop_pid.output_limits = (0, 2.0)
        position_loop_pid.setpoint = target_height
        target_velocity = position_loop_pid(self.motion_pose.pose.position.z)
        # 速度环
        velocity_loop_pid = PID(0.9, 0.01, 0.01, setpoint=target_velocity)
        velocity_loop_pid.output_limits = (0, 0.15)
        velocity_loop_pid.setpoint = target_velocity
        self.throttle = velocity_loop_pid(self.motion_velocity.twist.linear.z) + 0.15

        print("target_velocity:", target_velocity)
        print("throttle:", self.throttle)

    def callback_attitudemode(self):
        while not rospy.is_shutdown():
            if not self.check_position():
                return
            q = tf_conversions.transformations.quaternion_from_euler(self.roll, self.pitch, 0)
            self.set_attitude.orientation.x = q[0]
            self.set_attitude.orientation.y = q[1]
            self.set_attitude.orientation.z = q[2]
            self.set_attitude.orientation.w = q[3]
            self.pub_attitude.publish(self.set_attitude)

    def callback_state(self, msg):
        self.current_state = msg

    def callback_dyn(self, config, level):
        rospy.loginfo("""Reconfigure Request: {takeoff}""".format(**config))
        self.safe_x_max = config["safe_x_max"]
        self.safe_x_min = config["safe_x_min"]
        self.safe_y_max = config["safe_y_max"]
        self.safe_y_min = config["safe_y_min"]
        self.safe_z_max = config["safe_z_max"]
        self.safe_z_min = config["safe_z_min"]
        self.roll = config["roll"] / 180 * math.pi
        self.pitch = config["pitch"] / 180 * math.pi
        self.yaw = config["yaw"] / 180 * math.pi
        self.set_attitude.thrust = config["thrust"]
        self.set_attitude.body_rate.z = config["yaw_rate"] / 180 * math.pi

        self.set_velocity.twist.linear.x = config["velocity_x"]
        self.set_velocity.twist.linear.y = config["velocity_y"]
        self.set_velocity.twist.linear.z = config["velocity_z"]

        self.set_pose.pose.position.x = config["position_x"]
        self.set_pose.pose.position.y = config["position_y"]
        self.set_pose.pose.position.z = config["position_z"]

        self.mode = config["mode"]
        self.quit = config["QUIT"]
        self.attitude_mode = config["ATTITUDE_MODE"]
        self.takeoff = config["takeoff"]
        return config


if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    test = UAVController()
    test.spin()
