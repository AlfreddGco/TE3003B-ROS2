import rclpy
import pdb
from puzzlebot import Puzzlebot

rclpy.init()

robot = Puzzlebot()

while True:
    try:
        rclpy.spin_once(robot) 
        pdb.set_trace()
    except KeyboardException:
        break
