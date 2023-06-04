import pdb

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .puzzlebot_camera import PuzzlebotCamera
from .aruco_detection import ArucoDetection
from puzzlebot_control import VelocityBroadcaster

class Puzzlebot(Node):
  def __init__(self):
    super().__init__('puzzlebot')
    self.control = VelocityBroadcaster(self)
    self.camera = PuzzlebotCamera(self)
    self.arucos_detect = ArucoDetection(self.camera)
    self.create_timer(1/5, self.run)


  def run(self):
    # pdb.set_trace()
    self.arucos_detect.detect()


def main(args=None):
  rclpy.init(args=args)
  node = Puzzlebot()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()

