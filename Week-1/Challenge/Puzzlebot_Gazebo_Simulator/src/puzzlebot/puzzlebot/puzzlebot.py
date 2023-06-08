import pdb
import numpy as np
import math
import warnings

import struct, serial

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from utils import *
from puzzlebot_camera import PuzzlebotCamera
from aruco_detection import ArucoDetection, Aruco
from lidar import LidarSensor
from pose_calculation import Odometry
from control_node import VelocityBroadcaster
from position_control import PositionControl

from puzzlebot_planning.bug_0 import BugZero

LOAD_ZONE_MAP = {
    '0': '5'
}

DEFAULT_FD = "/dev/ttyUSB3"

class States:
    SEARCHING_LOAD = 0x11
    GOING_TO_LOAD = 0x22
    PICKING_LOAD = 0x33
    DELIVERING_LOAD = 0x44
    DROPPING_LOAD = 0x55


class Puzzlebot(Node):
  def __init__(self):
    super().__init__('puzzlebot')
    self.control = VelocityBroadcaster(self)
    self.camera = PuzzlebotCamera()
    self.lidar = LidarSensor(self)
    self.odom = Odometry(self, self.control)
    self.bug = BugZero(0.4)
    self.position_control = PositionControl(self.control, self.odom)
    self.rearDistance = 0.13
    self.loads = [
        Aruco(0),
        Aruco(1),
        Aruco(2),
        Aruco(3),
        Aruco(4),
    ]
    self.zones = [
        Aruco(5, (0, 0)),
        Aruco(6, (0, 0)),
        Aruco(7, (0, 0)),
    ]
    self.arucosDetection = ArucoDetection(self, self.camera,
        self.loads + self.zones, self.odom)
    self.priority = None 
    self.state = States.SEARCHING_LOAD
    self.rate = self.create_rate(50)

  def destroy_node(self):
    self.control.stop()
    self.control.broadcast_vel()
    self.camera.release()
    super().destroy_node()


  def set_goal(self, x, y):
    self.position_control.set_goal(x, y)
    self.bug.set_goal(self.position_control.goal)


  def wait_for_bug(self):
    while not self.position_control.on_goal():
      self.rate.sleep()
      # print(self.lidar.data[0][1])
      step = self.bug.next_step(
        self.odom.position, self.odom.theta, self.lidar.data)
      if(self.bug.circumnavigating):
        # direction relative to lidar
        step = rotate_vec(step, -90 + self.odom.theta * 180 / np.pi)
      print('VEL:', self.control.v, self.control.w, 'STEP:', step, self.lidar.data[0][1])
      # self.position_control.publish_direction(step)

    
  def find_aruco(self):
    while self.priority is None:
      self.rotate_puzzlebot()
      for load in loads:
        if (not load.collected and load.position is not None):
            self.priority = load
      

  # May implement in positional_control
  def puzzlebot_move(self, v, w):
      self.control.v = v
      self.control.w = w

  def close_gripper(self): 
    self.puzzlebot_move(0.1, 0.0)
    ser = serial.Serial(fd)
    ser.write(b'0x11')
    self.puzzlebot_move(-0.1, 0.0)
    self.rotate()

  
  def get_load_zone(self, load):
    zoneId = LOAD_ZONE_MAP[load.id]    
    for zone in self.zones:
      if (zone.id == zone_id):
          return zone
    raise ValueError('Zone not found in get_load_zone')


  def deploy_load(self): 
    ser = serial.Serial(fd)
    ser.write(b'0x12')
    self.puzzlebot_move(-0.1, 0.0)
    self.rotate()


  def missing_loads(self):
    c = 0
    for load in self.loads:
      if(not load.collected):
        c += 1
    return c


  def run(self):
    while len(self.lidar.data) == 0:
      warnings.warn('No lidar data yet...')
      self.rate.sleep()

    # self.control.v = 0.08
    #while True:
    #  self.rate.sleep()

    print('Starting bug...')
    self.set_goal(0, 1.2)
    self.wait_for_bug()
    return

    while self.missing_loads() > 0:
      self.find_aruco()
      # Go to aruco (TODO: calculate right position)
      current_position = self.set_goal(
          self.priority.position[0], self.priority.position[1])
      self.wait_for_bug()
      self.close_gripper() # Collect load
      zone = self.get_load_zone(self.priority) # Go to its zone
      self.set_goal(zone.position[0], zone.position[1])
      self.wait_for_bug()
      self.deploy_load()
      # Go to next not collected aruco, repeat bucle

    

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

