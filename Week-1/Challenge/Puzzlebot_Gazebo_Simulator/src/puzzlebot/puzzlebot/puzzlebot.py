import pdb

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from puzzlebot_control import PuzzlebotControl

class Puzzlebot(Node):
  def __init__(self):
    super().__init__('puzzlebot')
    self.control = PuzzlebotControl(self)
    self.create_timer(1, self.run)

  def run(self):
    pdb.set_trace()
    print(self.control.wl, self.control.wr)


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

