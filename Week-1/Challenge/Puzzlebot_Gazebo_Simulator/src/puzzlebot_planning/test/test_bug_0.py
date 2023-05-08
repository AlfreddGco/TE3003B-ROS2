from puzzlebot_planning.bug_0 import *


def test_goal_on_sight():
  bug = BugZero(3)
  bug.set_goal(np.array([7, 0]))
  on_sight = bug._goal_on_sight(np.array([0, 0]), np.array([[3, 0]]))
  assert on_sight == False


def test_get_boundary_step():
  bug = BugZero(3)

  positions = [np.array([0, 0])]
  boundaries = np.array([[-1, 4], [0, 3], [3, 3]])

  def update_positions(positions, boundaries, movement):
    positions.append(positions[-1] + movement)
    boundaries = boundaries - positions[-1]
    return positions, boundaries

  movement = bug._get_boundary_step(
    positions[-1], boundaries)

  assert movement[0] == 2.1213203435596424 and movement[1] == 0
  positions, boundaries = update_positions(
    positions, boundaries, movement)

  movement = bug._get_boundary_step(positions[-1], boundaries)
  assert movement[0] == -3 and movement[1] == 0

  positions, boundaries = update_positions(
    positions, boundaries, movement)

