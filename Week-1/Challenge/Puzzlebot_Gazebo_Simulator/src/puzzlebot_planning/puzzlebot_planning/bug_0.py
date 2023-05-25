import pdb
from math import cos, sin, atan2

import numpy as np

DEBUG = False

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


class BugZero:
  def __init__(self, radius):
    self.radius = radius
    self.circumnavigating = False
    self.goal = None


  def _goal_on_sight(self, position, orientation, boundaries):
    v = self.radius*unit_vector(self.goal - position)
    perpen = rotate_vec(v, 90)
    left_line = linear_eq(position + perpen, self.goal + perpen)
    right_line = linear_eq(position - perpen, self.goal - perpen)
    boundaries = self._clear_boundaries(boundaries)
    for boundary in boundaries:
      # no points in opposite direction
      if(boundary[0]*v[0] < 0 and boundary[1]*v[1] < 0):
        continue
      dist = np.linalg.norm(boundary)
      point = position + rotate_vec(
          boundary, -90 + orientation*180/np.pi)
      in_path = between_lines(left_line, right_line, point)
      if(in_path and dist < self.radius*1.1):
          return False
    return True


  def _clear_boundaries(self, boundaries):
    mask = np.isfinite(boundaries)
    return boundaries[np.logical_and(mask[:,0], mask[:,1])]


  def _closest_boundary(self, boundaries):
    distances = np.linalg.norm(boundaries, axis = 1)
    idx = list(distances).index(min(distances))
    return idx, boundaries[idx]


  def _is_colliding(self, boundaries):
    distances = np.linalg.norm(boundaries, axis = 1)
    return min(distances) <= self.radius


  def _get_boundary_step(self, boundaries):
    global DEBUG
    # no nan's nor inf's
    boundaries = self._clear_boundaries(boundaries)
    idx, closest = self._closest_boundary(boundaries)
    left_closest = boundaries[(idx + 1) % len(boundaries)]
    dist_to_closest = np.linalg.norm(closest)
    D = left_closest - closest
    R = closest
    RD_angle = angle_between(R, D)
    if(DEBUG):
      pdb.set_trace()
    if dist_to_closest > self.radius:
      return unit_vector(rotate_vec(closest, 90))
    if RD_angle > (np.pi/2)*(0.98) and RD_angle < (np.pi/2)*1.02:
      return unit_vector(rotate_vec(closest, 90))
    no_intersect = (self.radius - dist_to_closest)
    getaway = unit_vector(rotate_vec(closest, 180))*no_intersect
    getaway += unit_vector(rotate_vec(closest, 90))*no_intersect
    return getaway


  def _get_goal_step(self, position):
    return unit_vector(self.goal - position)


  def set_goal(self, position):
    self.goal = position


  def next_step(self, position, orientation, boundaries):
    if self.goal is None:
      raise Exception('goal is not set in bug object. '
        + 'Please set it with set_goal')
    if self._is_colliding(boundaries) or self.circumnavigating:
      if self._goal_on_sight(position, orientation, boundaries):
        self.circumnavigating = False
      else:
        self.circumnavigating = True
        return self._get_boundary_step(boundaries)
    return self._get_goal_step(position)

