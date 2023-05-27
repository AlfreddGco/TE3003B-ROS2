#!/usr/bin/env python3
import pdb, math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from tf_transformations import euler_from_quaternion

from puzzlebot_planning.bug_0 import *

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped

TOPIC_CALCULATED_POSE = '/gt_pose'
TOPIC_LIDAR_SCAN = '/scan'
TOPIC_VEL_CMD = '/cmd_vel'

DEBUG = False

def minimize_angle(angle):
  if(angle > np.pi):
    angle = angle - 2*np.pi
  elif(angle < -np.pi):
    angle = 2*np.pi + angle
  return angle


class PuzzlebotBug(Node):
  def __init__(self):
    super().__init__('puzzlebot_bug_zero')
    self.get_logger().info('init...')
    self.position = np.array([0, 0])
    self.orientation = 0

    self.goal = np.array([1.7, 4])
    self.lidar_data = np.array([])

    self.bug = BugZero(0.3)
    self.bug.set_goal(self.goal)

    self.create_subscription(
        LaserScan, TOPIC_LIDAR_SCAN, self.set_lidar_data, 10)
    self.create_subscription(
        PoseStamped, TOPIC_CALCULATED_POSE, self.set_position, 10)
    self.pub_cmd_vel = self.create_publisher(
        Twist, TOPIC_VEL_CMD, 10)


  def set_lidar_data(self, msg: LaserScan):
    ranges = msg.ranges
    l = len(msg.ranges)
    vectors = []
    for idx, d in enumerate(ranges):
      rad = (idx / l)*2*np.pi
      vectors.append([d*-sin(rad), d*cos(rad)])
    self.lidar_data = np.array(vectors)
  

  def set_position(self, msg: PoseStamped):
    self.position = np.array([msg.pose.position.x, msg.pose.position.y])
    orientation = msg.pose.orientation
    quats = [orientation.x, orientation.y, orientation.z, orientation.w]
    eulers = euler_from_quaternion(quats)
    yaw = eulers[2]
    if(yaw < 0):
        yaw = 2*np.pi + yaw
    self.orientation = yaw


  def publish_direction(self, direction):
    # direction = rotate_vec(direction, self.orientation*180/np.pi)
    target_angle = math.atan2(direction[1], direction[0])
    if(target_angle < 0):
      target_angle = 2*np.pi + target_angle
    angle_error = minimize_angle(target_angle - self.orientation)
    self.get_logger().info(
      'target angle: %.2f' % (target_angle*180/np.pi) +
      ' orientation: %.2f' % (self.orientation*180/np.pi) +
      ' error angle: %.2f' % (angle_error*180/np.pi)
    )
    twist_msg = Twist()
    if abs(angle_error) < (2*np.pi)*0.003:
      twist_msg.linear.x = 0.12
      twist_msg.angular.z = 0.0
    else:
      sign = (+1 if angle_error > 0 else -1)
      factor = (0.7/1.2)*(abs(angle_error)/np.pi + 0.2)
      factor = max(0.2, factor)
      factor = min(0.7, factor)
      twist_msg.linear.x = 0.03*(2/factor)/5
      twist_msg.angular.z = factor*sign
    self.pub_cmd_vel.publish(twist_msg)


  def stop(self):
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    self.pub_cmd_vel.publish(twist_msg)


  def on_goal(self):
    err = np.linalg.norm(self.goal - self.position)
    return err < 0.1


  def run(self):
    global DEBUG
    rate = self.create_rate(100)
    while not self.on_goal():
      rate.sleep()
      if DEBUG:
        self.stop()
        rate.sleep()
      if len(self.lidar_data) == 0:
        continue
      step = self.bug.next_step(
        self.position, self.orientation, self.lidar_data)
      if(self.bug.circumnavigating):
        # direction relative to lidar
        step = rotate_vec(step, -90 + self.orientation*180/np.pi)
      self.publish_direction(step)


def main(args = None):
  rclpy.init(args = args)
  executor = MultiThreadedExecutor()
  node = PuzzlebotBug()
  executor.add_node(node)
  task = executor.create_task(node.run)
  try:
    executor.spin_until_future_complete(task)
  except (KeyboardInterrupt, ExternalShutdownException):
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()
  finally:
    node.stop()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()
