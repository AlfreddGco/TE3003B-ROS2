import numpy as np
from math import sin, cos
from utils import implement_node

import rclpy
from rclpy.node import Node
from threading import Thread

from sensor_msgs.msg import LaserScan

class LidarSensor:
    def __init__(self, nh):
        implement_node(self, nh)
        self.data = np.array([])
        self.create_subscription(
            LaserScan, "/scan", self.set_lidar_data, 10)

    def set_lidar_data(self, msg: LaserScan):
        ranges = msg.ranges
        l = len(msg.ranges)
        vectors = []
        for idx, d in enumerate(ranges):
            rad = (idx / l)*2*np.pi
            vectors.append([d*-sin(rad), d*cos(rad)])
        self.data = np.array(vectors)


def target(node):
    while node.running:
        rclpy.spin_once(node)

if __name__ == '__main__':
    rclpy.init()
    node = Node('lidar_test')
    node.running = True
    sensor = LidarSensor(node)

    spin_task = Thread(target=target, args=(node,))
    spin_task.start()
    def finish():
        node.running = False
        spin_task.join()
        exit()

