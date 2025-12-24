import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from .ga_math import Rotor2D


class GAMotionNode(Node):

    def __init__(self):
        super().__init__('ga_motion_node')

        self.dt = 0.05
        self.pose = np.array([0.0, 0.0])
        self.theta = 0.0

        self.v = 0.0
        self.omega = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.path = Path()
        self.path.header.frame_id = 'map'

        self.timer = self.create_timer(self.dt, self.update)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update(self):
        self.theta += self.omega * self.dt

        rotor = Rotor2D(self.theta)
        direction = rotor.rotate([1.0, 0.0])
        self.pose += self.v * direction * self.dt

        # Pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(self.pose[0])
        pose_msg.pose.position.y = float(self.pose[1])
        pose_msg.pose.orientation.z = np.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = np.cos(self.theta / 2.0)

        self.pose_pub.publish(pose_msg)

        # Path
        self.path.header.stamp = pose_msg.header.stamp
        self.path.poses.append(pose_msg)
        self.path_pub.publish(self.path)

        # TF
        tf = TransformStamped()
        tf.header = pose_msg.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = pose_msg.pose.position.x
        tf.transform.translation.y = pose_msg.pose.position.y
        tf.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    rclpy.spin(GAMotionNode())
    rclpy.shutdown()
