import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np
import math

from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from std_msgs.msg import Float32

from utils import implement_node, quaternion_from_euler

TOPIC_CALCULATED_POSE = '/calculated_pose'

class GazeboPoseBroadcaster:
    def __init__(self):
        self.create_subscription(
            ModelStates, '/model_states', self.publish_gazebo_pose, 10)
        self.gt_publisher = self.create_publisher(
            PoseStamped, '/gt_pose', 10)


    def publish_gazebo_pose(self, msg: ModelStates):
      idx = msg.name.index('puzzlebot')
      if(idx == -1):
        self.get_logger().warn('Puzzlebot not found in msg.name')
        return
      gt_msg = PoseStamped()
      gt_msg.header.stamp = self.get_clock().now().to_msg()
      gt_msg.pose = msg.pose[idx]
      self.gt_publisher.publish(gt_msg)
    



class Odometry:
    def __init__(self, nh, control):
        implement_node(self, nh)
        self.x, self.y, self.theta = 0.0, 0.0, np.pi/2
        self.wl, self.wr = 0, 0
        self.t = self.get_time()
        self.filtered_position = np.array([0, 0])
        self.control = control

        #self.create_subscription(Float32, '/wl', self.update_wl, 10)
        #self.create_subscription(Float32, '/wr', self.update_wr, 10)
        self.create_timer(1/50, self.calculate_position)


    def update_wl(self, msg):
        self.wl = msg.data


    def update_wr(self, msg):
        self.wr = msg.data


    def get_time(self):
        return self.nh.get_clock().now().nanoseconds / 1e9


    @property
    def position(self):
        return np.array([self.x, self.y])


    def calculate_position(self):
        #L, R = 0.08, 0.05
        #MAT = np.array([[.5, .5], [1/L, -1/L]])
        #dv, dw = np.dot(MAT, np.array([self.wr, self.wl])*R)
        now = self.get_time()
        dt = now - self.t
        dv = self.control.v
        dw = self.control.w
        self.x += (dv * math.cos(self.theta))*dt
        self.y += (dv * math.sin(self.theta))*dt
        self.theta += dw*dt
        self.t = now



class MainNode(Node):
    def __init__(self):
        super().__init__('pose_calculation')
        self.pose = Odometry(self)
        self.create_timer(1/30, self.publish_pose_and_transform)
        self.reset_service = self.create_service(
            Empty, 'reset_pose_calculation', self.reset_state)
        self.publisher_pose = self.create_publisher(
            PoseStamped, TOPIC_CALCULATED_POSE, 10) 


    def publish_pose_and_transform(self):
        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = 'base_link'

        stamped_pose.pose.position.x = self.pose.x
        stamped_pose.pose.position.y = self.pose.y

        # TODO: Do direct calculation only with yaw
        q = quaternion_from_euler(0, 0, self.pose.theta)
        stamped_pose.pose.orientation.x = q[0]
        stamped_pose.pose.orientation.y = q[1]
        stamped_pose.pose.orientation.z = q[2]
        stamped_pose.pose.orientation.w = q[3]
        self.publisher_pose.publish(stamped_pose)


    def reset_state(self, _, res):
        self.get_logger().info('Resetting state')
        self.pose.x, self.pose.y = 0.0, 0.0
        self.pose.theta = 0.0
        return res



def main():
    rclpy.init()
    node = MainNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
