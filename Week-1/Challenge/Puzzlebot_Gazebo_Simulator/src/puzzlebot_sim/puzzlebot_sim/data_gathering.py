#!/usr/bin/env python3
import csv, time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from tf_transformations import euler_from_quaternion

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from tf2_geometry_msgs import PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState

from std_srvs.srv import Empty

TOPIC_CALCULATED_POSE = '/calculated_pose'
TOPIC_MODEL_STATE = '/model_states'
TOPIC_VEL_CMD = '/cmd_vel'

DATA_FILENAME = 'gathered_poses.csv'

class DataGathering(Node):
    def __init__(self):
        super().__init__('puzzlebot_data_gathering')
        self.get_logger().info('init...')
        self.rviz_pose, self.gazebo_pose = PoseStamped(), Pose()
        self.create_subscription(
            PoseStamped, TOPIC_CALCULATED_POSE, self.set_rviz_pose, 10)
        self.create_subscription(
            ModelStates, TOPIC_MODEL_STATE, self.update_gazebo_pose, 10)
        self.pub_cmd_vel = self.create_publisher(
            Twist, TOPIC_VEL_CMD, 10)
    
        self.get_logger().info('Waiting for reset service')
        self.reset_client = self.create_client(Empty, 'reset_pose_calculation')
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Reset service available')


        self.restart_gazebo_srv = self.create_client(SetEntityState, '/set_entity_state')
        while not self.restart_gazebo_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Reset service available')

        self.reset_req = Empty.Request()
        self.gazebo_reset_req = SetEntityState.Request()
        self.gazebo_reset_req.state.name = 'puzzlebot'

    
    def set_rviz_pose(self, pose: PoseStamped):
        self.rviz_pose = pose


    def update_gazebo_pose(self, msg: ModelStates):
        idx = msg.name.index('puzzlebot')
        self.gazebo_pose = msg.pose[idx]


    def save_data(self, writer: csv.writer):
        self.get_logger().info("New row into csv")
        angles_gazebo = euler_from_quaternion([
            self.gazebo_pose.orientation.x, self.gazebo_pose.orientation.y,
            self.gazebo_pose.orientation.z, self.gazebo_pose.orientation.w])
        writer.writerow([
            self.gazebo_pose.position.x,
            self.gazebo_pose.position.y,
            angles_gazebo[2]])


    def reset_environment(self):
        self.get_logger().info('Resetting environment...')
        self.reset_client.call(self.reset_req)
        self.restart_gazebo_srv.call(self.gazebo_reset_req)
        self.get_logger().info('Environment has reset')


    def publish_twist_msgs(self, x, z):
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.angular.z = z
        self.pub_cmd_vel.publish(twist_msg)


    def sleep(self, period):
        rate = self.create_rate(1/period)
        rate.sleep()
        rate.sleep()


    def run(self):
        self.reset_environment()
        self.get_logger().info('Running data gathering routine')
        with open(DATA_FILENAME, 'w', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(['gazebo_x', 'gazebo_y', 'gazebo_t'])
            for _ in range(12):
                self.publish_twist_msgs(0.5, 0.0)
                self.get_logger().info('publish twist')
                self.sleep(2)
                self.publish_twist_msgs(0.0, 0.0)
                self.get_logger().info('robot stopperd')
                self.sleep(1)
                self.save_data(writer)
                self.reset_environment()

            for _ in range(12):
                self.publish_twist_msgs(0.0, 1.0)
                self.sleep(2)
                self.publish_twist_msgs(0.0, 0.0)
                self.sleep(1)
                self.save_data(writer)
                self.reset_environment()
        f.close()


def main(args = None):
    rclpy.init(args = args)
    executor = MultiThreadedExecutor()
    node = DataGathering()
    executor.add_node(node)
    task = executor.create_task(node.run)
    try:
        executor.spin_until_future_complete(task)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()    
