import numpy as np
import rospy
import csv

import geometry_msgs.msg
from mavros_msgs.msg import BatteryStatus, ServoOutput, AttitudeTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
import tf_conversions
import time
import tf

sub_battery_topic = 'mavros/battery'
sub_odom_topic = 'px4/vision_odom'
sub_imu_topic = "mavros/imu/data"
sub_pwm_topic = "mavros/servo_output_raw"
sub_attitude_topic = "mavros/setpoint_raw/attitude"
num_count = 0
csv_topics = ['num', 't', 'p', 'v', 'q', 'T_sp', 'q_sp', 'hover_throttle', 'fa', 'pwm']
data_storage = []
dict_interface = {i: 0 for i in csv_topics}
fa = []
a = [0, 0, 0]
pwm = []
weight = 1.12
g = 9.81
# init node
rospy.init_node('data_capture_sync')
hover_throttle = 0.53
start_t = 0
data_odom = Odometry()
data_pwm = ServoOutput()
data_imu = Imu()
data_battery = BatteryStatus()
data_battery.voltage = 12.6
data_attitude = AttitudeTarget()


def battery_cb(data: BatteryStatus):
    data_battery.voltage = data.voltage


def imu_cb(data: Imu):
    data.orientation = data.orientation


def attitude_cb(data: AttitudeTarget):
    data_attitude.orientation = data.orientation
    data_attitude.thrust = data.thrust


def odom_cb(data: Odometry):
    global num_count, start_t
    if data_odom.pose.pose.position.x == 0:
        start_t = data.header.stamp.to_sec()
        pass
    else:
        dt = data.header.stamp.to_sec() - data_odom.header.stamp.to_sec()
        a[0] = (data.twist.twist.linear.x - data_odom.twist.twist.linear.x) / dt * weight
        a[1] = (data.twist.twist.linear.y - data_odom.twist.twist.linear.y) / dt * weight
        a[2] = (data.twist.twist.linear.z - data_odom.twist.twist.linear.z) / dt * weight
        maritx = tf_conversions.transformations.quaternion_matrix([data_attitude.orientation.x,
                                                                   data_attitude.orientation.y,
                                                                   data_attitude.orientation.z,
                                                                   data_attitude.orientation.w])
        T = (data_attitude.thrust + 0.12) / 0.05647
        Z = np.array([[0], [0], [1], [0]])
        # rospy.loginfo((T * maritx @ Z).flatten()[0:3])

        faa = ((T * maritx @ Z).flatten()[0:3] - np.array(a)).T
        fa = faa.tolist()
        fa[2] = fa[2] - weight * g
        dict_interface['fa'] = fa
        dict_interface['hover_throttle'] = hover_throttle
        dict_interface['pwm'] = pwm
        dict_interface['p'] = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        dict_interface['v'] = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
        dict_interface['q'] = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        dict_interface['T_sp'] = data_attitude.thrust
        dict_interface['q_sp'] = [data_attitude.orientation.x, data_attitude.orientation.y,
                                  data_attitude.orientation.z, data_attitude.orientation.w]
        dict_interface['num'] = num_count
        dict_interface['t'] = data.header.stamp.to_sec() - start_t
        num_count += 1
        data_storage.append(dict_interface.copy())

    data_odom.pose = data.pose
    data_odom.twist = data.twist


def pwm_cb(data: ServoOutput):
    global pwm
    data_pwm.controls = data.controls
    pwm = [data_pwm.controls[0], data_pwm.controls[1], data_pwm.controls[2], data_pwm.controls[3]]


def to_csv(output_file_name: str):
    with open(output_file_name, 'w',encoding='utf8', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_topics)
        writer.writeheader()
        writer.writerows(data_storage)

sub_odom = rospy.Subscriber(sub_odom_topic, Odometry, odom_cb)
sub_imu = rospy.Subscriber(sub_imu_topic, Imu, imu_cb)
sub_pwm = rospy.Subscriber(sub_pwm_topic, ServoOutput, pwm_cb)
sub_battery = rospy.Subscriber(sub_battery_topic, BatteryStatus, battery_cb)
sub_attitude = rospy.Subscriber(sub_attitude_topic, AttitudeTarget, attitude_cb)

topic_names = []
csv_names = []

output_file_name = "/home/hoshino/data/neural-fly/data/t2_experiment/custom_random10_mpc_2832v.csv"
rospy.sleep(90)
to_csv(output_file_name)
