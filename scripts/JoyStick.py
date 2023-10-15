#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from math import pi
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped,TwistStamped,PointStamped
from simple_pid import PID
# from uav_control.msg import Test

class JoyStick(object):
    use_attitude = True
    """---------------------------------------"""
    max_throttle = 0.48
    min_throttle = 0.38
    middle_throttle = 0.43
    """---------------------------------------"""
    vertical_valve = 0
    forward_value = 0
    turn_value = 0
    side_value = 0
    forward_button = 0
    side_button = 0
    turn_button = 0
    """-----------------------------------------"""
    # 速度
    vertical_vabs = 0.3
    horizon_vabs = 0.3
    turn_vabs = 0.3
    time_step = 0.1
    """-----------------------------------------"""
    # 位置，步进
    vertical_sabs = 0.1
    horizon_sabs = 0.1
    turn_sabs = 5  # 5度
    """-----------------------------------------"""
    # 位置绝对位置
    vertical_abs = 0
    forward_abs = 0
    side_abs = 0
    turn_abs = 0
    """-----------------------------------------"""
    throttle = 0
    roll_command = 0
    pitch_command = 0
    yaw_rate_command = 0
    roll_max = 20  # deg
    pitch_max = 20  # deg
    yaw_rate_max = 5  # deg/s
    """-----------------------------------------"""
    z_max = 0
    z_min = 0
    x_max = 0
    x_min = 0
    y_max = 0  # deg
    y_min = 0  # deg
    """-----------------------------------------"""

    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('joystick_listener', anonymous=True)

        self.motion_pose = PoseStamped()
        self.motion_velocity = TwistStamped()
        self.yaw_command = 0

        # 订阅Joystick的消息
        if self.use_attitude:
            rospy.Subscriber("/joy", Joy, self.joystick_callback_attitude, tcp_nodelay=True)
        else:
            rospy.Subscriber("/joy", Joy, self.joystick_callback_pose, tcp_nodelay=True)

        # 位置sub，回调函数记录位置并转换单位
        self.sub_pose = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback=self.callback_pose,
                                         tcp_nodelay=True)
        # 速度sub，回调函数记录位置并转换单位
        self.sub_velocity = rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, callback=self.callback_velocity,
                                         tcp_nodelay=True)
        
        self.attitude_publisher = rospy.Publisher("/mavros/setpoint_raw/attitude", data_class=AttitudeTarget
                                                  , tcp_nodelay=True, queue_size=1)
        self.pose_publisher = rospy.Publisher("/uav/command/pose", data_class=PoseStamped
                                              , tcp_nodelay=True, queue_size=1)
        
        self.test_publisher = rospy.Publisher("/mavros/setpoint_raw/test", data_class=PointStamped
                                                  , tcp_nodelay=True, queue_size=1)

    def callback_pose(self, pose_data: PoseStamped):
        # 回调函数，记录实际位置值并进行单位转换，mm->m
        self.motion_pose.pose.position.x = pose_data.pose.position.x/1000
        self.motion_pose.pose.position.y = pose_data.pose.position.y/1000
        self.motion_pose.pose.position.z = pose_data.pose.position.z/1000
        # self.motion_pose.pose.position.x = pose_data.pose.position.x
        # self.motion_pose.pose.position.y = pose_data.pose.position.y
        # self.motion_pose.pose.position.z = pose_data.pose.position.z
        self.motion_pose.pose.orientation = pose_data.pose.orientation

    def callback_velocity(self, velocity_data: TwistStamped):
        # 回调函数，记录实际位置值并进行单位转换，mm->m
        self.motion_velocity.twist.linear.x = velocity_data.twist.linear.x/1000
        self.motion_velocity.twist.linear.y = velocity_data.twist.linear.y/1000
        self.motion_velocity.twist.linear.z = velocity_data.twist.linear.z/1000
        self.motion_velocity.twist.angular.x = velocity_data.twist.angular.x/1000
        self.motion_velocity.twist.angular.y = velocity_data.twist.angular.y/1000
        self.motion_velocity.twist.angular.z = velocity_data.twist.angular.z/1000
        # self.motion_velocity.twist.linear.x = velocity_data.twist.linear.x
        # self.motion_velocity.twist.linear.y = velocity_data.twist.linear.y
        # self.motion_velocity.twist.linear.z = velocity_data.twist.linear.z
        # self.motion_velocity.twist.angular.x = velocity_data.twist.angular.x
        # self.motion_velocity.twist.angular.y = velocity_data.twist.angular.y
        # self.motion_velocity.twist.angular.z = velocity_data.twist.angular.z

    def publish_attitude(self):
        pub_raw = AttitudeTarget()
        pub_rpy = PointStamped()
        pub_raw.type_mask = 3
        pub_raw.header.stamp = rospy.rostime.get_rostime()
        pub_raw.body_rate.z = 0
        self.yaw_command = self.yaw_command + self.yaw_rate_command
        [pub_raw.orientation.x, pub_raw.orientation.y, pub_raw.orientation.z,
         pub_raw.orientation.w] = quaternion_from_euler(ai=self.roll_command, aj=self.pitch_command, ak=self.yaw_command)
        pub_raw.thrust = self.throttle
        self.attitude_publisher.publish(pub_raw)

        pub_rpy.header.stamp = rospy.rostime.get_rostime()
        pub_rpy.point.x, pub_rpy.point.y, pub_rpy.point.z = self.roll_command, self.pitch_command, self.yaw_command
        self.test_publisher.publish(pub_rpy)

    def publish_pose(self):
        pub_raw = PoseStamped()
        pub_raw.header.stamp = rospy.rostime.get_rostime()
        pub_raw.pose.position.x, pub_raw.pose.position.y, pub_raw.pose.position.z = (
            self.side_abs, self.forward_abs, self.vertical_abs)
        [pub_raw.pose.orientation.x, pub_raw.pose.orientation.y, pub_raw.pose.orientation.z,
         pub_raw.pose.orientation.w] = quaternion_from_euler(ai=0, aj=0, ak=0)
        self.pose_publisher.publish(pub_raw)

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
        self.throttle = velocity_loop_pid(self.motion_velocity.twist.linear.z)+0.15

        print("target_velocity:",target_velocity)
        print("throttle:",self.throttle)

    def joystick_callback_attitude(self, data: Joy):
        # 获取Joystick的输入值
        # self.throttle = (data.axes[1] + 1) / 2
        self.target_height = self.motion_pose.pose.position.z
        self.calculate_throttle(self.throttle)
        self.pitch_command = data.axes[2] * self.pitch_max / 180 * pi
        self.roll_command = - data.axes[3] * self.roll_max / 180 * pi
        self.yaw_rate_command = data.axes[0] * self.yaw_rate_max / 180 * pi

    def joystick_callback_pose(self, data: Joy):
        # 获取Joystick的输入值
        self.side_abs = (data.axes[2] + 1) / 2 * (self.x_max - self.x_min) + self.x_min
        self.forward_abs = (data.axes[3] + 1) / 2 * (self.y_max - self.y_min) + self.y_min
        self.vertical_abs = (data.axes[1] + 1) / 2 * (self.z_max - self.z_min) + self.z_min

    def calculate_throttle(self, throttle):
        if throttle >= 0.5:
            self.throttle = (throttle - 0.5) * 2 * (self.max_throttle - self.middle_throttle) + self.middle_throttle
        else:
            self.throttle = throttle * 2 * (self.middle_throttle - self.min_throttle) + self.min_throttle

    def joystick_callback_normal(self, data: Joy):
        # 获取Joystick的输入值
        axes = data.axes
        buttons = data.buttons

        # 在这里添加你的逻辑代码
        self.vertical_value = data.axes[1]
        self.forward_value = data.axes[3]
        self.side_value = data.axes[2]
        # self.turn_value = data.axes[0]
        # 例如，你可以检查按下的按钮和摇杆的位置，并执行相应的操作
        """-----------------------------------------"""
        self.side_button = data.axes[-2]
        self.forward_button = data.axes[-1]
        self.turn_button = - data.buttons[6] + data.buttons[7]

        """-----------------------------------------"""
        self.turn_abs += self.turn_button / 180 * pi
        self.vertical_abs += self.vertical_value * self.vertical_sabs

        """-----------------------------------------"""

        self.forward_abs += self.forward_value * self.horizon_vabs * self.time_step
        # or
        self.forward_abs += self.forward_button * self.horizon_sabs

        self.side_abs += self.side_value * self.horizon_vabs * self.time_step
        # or
        self.side_abs += self.side_button * self.horizon_sabs

        # 示例：打印按钮和摇杆的值
        print("Axes: ", axes)
        print("Buttons: ", buttons)
        print("-------------------------")

    @staticmethod
    def joystick_listener():
        # 循环等待回调函数
        rospy.spin()


if __name__ == '__main__':
    try:
        joystick = JoyStick()
        rate = rospy.Rate(50)
        while True:
            if joystick.use_attitude:
                joystick.fixed_height(0.8)
                joystick.publish_attitude()
            else:
                joystick.publish_pose()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
