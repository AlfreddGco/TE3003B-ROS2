import pdb
import numpy as np
import math
import warnings

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from utils import *
from puzzlebot_camera import PuzzlebotCamera
from aruco_detection import ArucoDetection
from lidar import LidarSensor
from pose_calculation import Odometry
from control_node import VelocityBroadcaster
from position_control import PositionControl

from puzzlebot_planning.bug_0 import BugZero

class Puzzlebot(Node):
  def __init__(self):
    super().__init__('puzzlebot')
    self.control = VelocityBroadcaster(self)
    self.camera = PuzzlebotCamera()
    self.arucos_detect = ArucoDetection(self.camera)
    self.lidar = LidarSensor(self)
    self.odom = Odometry(self)
    self.bug = BugZero(0.4)
    self.position_control = PositionControl(self.control, self.odom)


  def destroy_node(self):
    self.control.stop()
    self.control.broadcast_vel()
    self.camera.release()
    super().destroy_node()


  def run(self):
    self.position_control.set_goal(0, 1.8)
    self.bug.set_goal(self.position_control.goal)

    rate = self.create_rate(50)

    while len(self.lidar.data) == 0:
      warnings.warn('No lidar data yet...')
      rate.sleep()

    print('Starting up bug algo')
    while not self.position_control.on_goal():
      rate.sleep()
      step = self.bug.next_step(
        self.odom.position, self.odom.theta,
        self.lidar.data)
      if(self.bug.circumnavigating):
        # direction relative to lidar
        step = rotate_vec(step, -90 + self.odom.theta*180/np.pi)
      print('VEL:', self.control.v, self.control.w, 'STEP:', step)
      self.position_control.publish_direction(step)


def main(args=None):
  rclpy.init(args=args)
  executor = MultiThreadedExecutor(num_threads=4)
  node = Puzzlebot()
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

