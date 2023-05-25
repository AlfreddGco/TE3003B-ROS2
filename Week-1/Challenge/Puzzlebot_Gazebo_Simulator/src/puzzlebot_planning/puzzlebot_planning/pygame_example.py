from bug_0 import *

import pygame

# pygame setup
pygame.init()

W, H = 1280, 680
screen = pygame.display.set_mode((W, H))
clock = pygame.time.Clock()
running = True

class Game:
  def __init__(self, screen):
    self.screen = screen
    self.h = screen.get_height()
    self.w = screen.get_width()
    self.target = (1200, 40)
    self.obstacles = np.array([
      [int(640 + i*10), int(340 - i*10)] for i in range(-3, 3)
      ])


  def draw_target(self):
    pygame.draw.circle(self.screen, 'blue', self.target, 10) 


  def draw_obstacles(self):
    for obs in self.obstacles:
      pos = (obs[0], self.h - obs[1])
      pygame.draw.circle(self.screen, 'black', pos, 2)
    pass


class Robot:
  def __init__(self, radius):
    self.radius = radius
    self.pos = np.array([575.29411765, 306.82352941])


  def display(self, screen):
    w, h = screen.get_width(), screen.get_height()
    pos = (self.pos[0], h - self.pos[1])
    pygame.draw.circle(screen, "red", pos, self.radius, 1)
    pygame.draw.circle(screen, "green", pos, 3)


  def get_lidar_data(self, obstacles):
    return obstacles - self.pos


game = Game(screen)
robot = Robot(20)
bug = BugZero(robot.radius)
bug.set_goal(np.array([
  game.target[0], H - game.target[1]]))

while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False

  screen.fill("white")
  robot.display(screen)
  game.draw_target()
  game.draw_obstacles()

  ### Draw width lines
  v = bug.radius*unit_vector(bug.goal - robot.pos)
  v_left, v_right = rotate_vec(v, 90), rotate_vec(v, -90)
  p1 = robot.pos + v_left
  p2 = bug.goal + v_left
  pygame.draw.line(screen, 'blue',
    (p1[0], H - p1[1]), [p2[0], H - p2[1]])
  p1 = robot.pos + v_right
  p2 = bug.goal + v_right
  pygame.draw.line(screen, 'red',
      (p1[0], H - p1[1]), [p2[0], H - p2[1]])
  ###

  step, circumnavigating = bug.next_step(
      robot.pos, robot.get_lidar_data(game.obstacles))
  robot.pos = np.sum([robot.pos, step], axis=0)

  pygame.display.flip()
  clock.tick(20)

pygame.quit()

