import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float64, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

TOPIC_WHEELS_VEL_CMD = '/velocity_controller/commands'
# TODO: Load from yaml controller file
TOPIC_JOINTS_STATE = '/joint_states'
# wheels
JOINTS = [
    { "name": "base_to_left_w", "publish_velocity": "wl" },
    { "name": "base_to_right_w", "publish_velocity": "wr" }
]

TOPIC_ROBOT_VELOCITY_CMD = '/cmd_vel'

# TODO: Pass through yaml (diff_controller params)
L = 0.191
R = 0.05

class PuzzlebotControl(Node):
  def __init__(self, joints = JOINTS):
    super().__init__('puzzlebot_control_node')
    # velocities unwrapping
    self.create_subscription(JointState, TOPIC_JOINTS_STATE,
      self.unwrap_joint_states, 10)
    self.init_joints(joints)

    # control
    self.publisher_wheels_cmd = self.create_publisher(
      Float64MultiArray, TOPIC_WHEELS_VEL_CMD, 10)
    self.create_subscription(Twist, TOPIC_ROBOT_VELOCITY_CMD,
      self.calculate_wheels_speed, 10)


  def init_joints(self, joints):
    self.joints = []
    for joint in joints:
      self.joints.append(joint)
      last = self.joints[-1]
      last["pub_unwrapped_vel"] = self.create_publisher(
        Float64, joint["publish_velocity"], 10)


  def calculate_wheels_speed(self, twist_vel):
      v = twist_vel.linear.x
      w = twist_vel.angular.z
      wl = (v -  w * L / 2.0) / R
      wr = (v + w * L / 2.0) / R
      cmd_vels = Float64MultiArray()
      cmd_vels.data.append(wl)
      cmd_vels.data.append(wr)
      self.publisher_wheels_cmd.publish(cmd_vels)


  def unwrap_joint_states(self, state):
    names = state.name
    for joint in self.joints:
      idx = names.index(joint["name"])
      vel_msg = Float64()
      vel_msg.data = state.velocity[idx]
      joint["pub_unwrapped_vel"].publish(vel_msg)


def main(args=None):
  rclpy.init(args=args)
  node = PuzzlebotControl()
  try:
    rclpy.spin(node)
  except (KeyboardInterrupt, ExternalShutdownException):
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()

