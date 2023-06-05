import os, time
import cv2
from threading import Thread

def get_pipeline():
    username = os.getlogin()
    if(username == 'jetson'):
        return 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)480,format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink'
    else:
        return 0


class PuzzlebotCamera:
    def __init__(self):
        self.frame = None
        self.cap = cv2.VideoCapture(get_pipeline())
        # create another thread for updating frame
        # with self.update_frame every 1/60 secs
        self.running = True
        self.updating_task = Thread(target=self._update_frames, args=())
        self.updating_task.start()


    def _update_frames(self):
        while self.running:
            _, self.frame = self.cap.read()
            # TODO: use better timer. Take into account exec time
            # of cap.read()
            time.sleep(1/70)


    def release(self):
        self.running = False
        self.updating_task.join()
        self.cap.release()
    

    def __del__(self):
        self.release()


if __name__ == '__main__':
    camera = PuzzlebotCamera()

