import numpy as np
import math
from utils import minimize_angle

class PositionControl:
  def __init__(self, control, odometry):
    # gives position and orientation
    self.control = control
    self.odom = odometry 
    self.goal = np.array([0, 0])
  

  def on_goal(self):
    dist = np.linalg.norm(self.goal - self.odom.position)
    return dist < 0.1


  def set_goal(self, x, y):
    self.goal[0], self.goal[1] = x, y


  def publish_direction(self, direction):
    orientation = self.odom.theta
    target_angle = math.atan2(direction[1], direction[0])
    if(target_angle < 0):
      target_angle = 2*np.pi + target_angle
    angle_error = minimize_angle(target_angle - orientation)
    print(
      'target angle: %.2f' % (target_angle*180/np.pi) +
      ' orientation: %.2f' % (orientation*180/np.pi) +
      ' error angle: %.2f' % (angle_error*180/np.pi)
    )
    if abs(angle_error) < (2*np.pi)*0.006:
      self.control.v = 0.15
      self.control.w = 0.0
    else:
      sign = (+1 if angle_error > 0 else -1)
      self.control.v = 0.05
      self.control.w = 0.10*sign

