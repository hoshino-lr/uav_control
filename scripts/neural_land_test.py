from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import tf_conversions
import rospy
import time


class NeualTest(object):

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

        self.quit = False

        # create parameter generator

        self.sub_pose = rospy.Subscriber(self.sub_pose_topic, PoseStamped, callback=self.callback_pose,
                                         tcp_nodelay=True)
        self.sub_state = rospy.Subscriber(self.sub_state_topic, State, callback=self.callback_state, tcp_nodelay=True)
        self.pub_pose = rospy.Publisher(self.pub_pose_topic, PoseStamped, queue_size=1, tcp_nodelay=True)
        self.set_mode_client = rospy.ServiceProxy(self.srv_mode_topic, SetMode)

        self.safe_x_max, self.safe_x_min, self.safe_y_max, self.safe_y_min, self.safe_z_max, self.safe_z_min = \
            0, 0, 0, 0, 0, 0

        time.sleep(1)
        self.pose_init()

    def pose_init(self):
        self.set_pose.pose.position.x = self.motion_pose.pose.position.x
        self.set_pose.pose.position.y = self.motion_pose.pose.position.y
        self.set_pose.pose.position.z = self.motion_pose.pose.position.z

    def land_mode(self):
        # land
        while not rospy.is_shutdown() and self.current_state.mode != "AUTO.LAND":
            self.set_mode_client(0, "AUTO.LAND")
            self.rate.sleep()
        rospy.loginfo("LAND success")

    def spin(self):
        while not rospy.is_shutdown() and not self.quit:
            if not self.check_position():
                self.land_mode()
                break
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            self.set_pose.pose.orientation.x = q[0]
            self.set_pose.pose.orientation.y = q[1]
            self.set_pose.pose.orientation.z = q[2]
            self.set_pose.pose.orientation.w = q[3]
            self.pub_pose.publish(self.set_pose)
            self.rate.sleep()

    def check_position(self) -> bool:
        if self.safe_x_max > self.motion_pose.pose.position.x > self.safe_x_min and \
                self.safe_y_max > self.motion_pose.pose.position.y > self.safe_y_min and \
                self.safe_z_max > self.motion_pose.pose.position.z > self.safe_z_min:
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
    test = NeualTest()
    test.spin()
