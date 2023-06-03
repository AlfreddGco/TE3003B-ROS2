import cv2
import time
from threading import Thread

def implement_node(obj, nh):
    obj.nh = nh
    obj.create_publisher = nh.create_publisher
    obj.create_subscription = nh.create_subscription
    obj.create_timer = nh.create_timer


class PuzzlebotCamera:
    def __init__(self, nh):
        implement_node(self, nh)
        self.frame = None
        self.cap = cv2.VideoCapture(0)
        # create another thread for updating frame
        # with self.update_frame every 1/60 secs
        self.updating_task = Thread(target=self.update_frames, args=())
        self.updating_task.start()

    def update_frames(self):
        while True:
            _, self.frame = self.cap.read()
            time.sleep(1/60)
    
    def __del__(self):
        self.updating_task.stop()
        self.cap.release()
