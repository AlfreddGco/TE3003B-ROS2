from math import cos, sin, atan2
import numpy as np

from rclpy.node import Node

def implement_node(obj, nh: Node):
    obj.nh = nh
    obj.create_publisher = nh.create_publisher
    obj.create_subscription = nh.create_subscription
    obj.create_timer = nh.create_timer


# TODO: Import from library
def quaternion_from_euler(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  return [qx, qy, qz, qw]


def minimize_angle(angle):
  if(angle > np.pi):
    angle = angle - 2*np.pi
  elif(angle < -np.pi):
    angle = 2*np.pi + angle
  return angle


def unit_vector(vector):
  """ Returns the unit vector of the vector.  """
  return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
  """ Returns the angle in radians between vectors 'v1' and 'v2'::
          >>> angle_between((1, 0, 0), (0, 1, 0))
          1.5707963267948966
          >>> angle_between((1, 0, 0), (1, 0, 0))
          0.0
          >>> angle_between((1, 0, 0), (-1, 0, 0))
          3.141592653589793
  """
  v1_u = unit_vector(v1)
  v2_u = unit_vector(v2)
  return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def rotate_vec(vec, deg):
  theta = np.deg2rad(deg)
  rotation = np.array([
    [cos(theta), -sin(theta)],
    [sin(theta), cos(theta)]
  ])
  return np.dot(rotation, vec)


def linear_eq(point_a, point_b):
  y, x = point_a[1], point_a[0]
  dx = point_b[0] - x
  if(dx == 0):
    return (float('inf'), x) 
  m = (point_b[1] - y) / dx 
  b = y - m*x
  return (m, b)


def between_lines(l1, l2, point):
  x, y = point
  if(l1[1] > l2[1]):
    m_l, b_l = l1
    m_r, b_r = l2
  else:
    m_l, b_l = l2
    m_r, b_r = l1
  # vertical lines
  if(m_l == float('inf') or m_r == float('inf')):
    if(x > b_l and x < b_r):
      return True
  below_left = (m_l*x + b_l) > y
  above_right = (m_r*x + b_r) < y
  if below_left and above_right:
    return True
  return False

