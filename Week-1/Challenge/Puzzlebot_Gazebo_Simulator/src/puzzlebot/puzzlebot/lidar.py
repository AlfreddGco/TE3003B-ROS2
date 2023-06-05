import numpy as np
from utils import implement_node

class LidarSensor:
    def __init__(self, nh):
        implement_node(self, nh)
        self.data = np.array([])

