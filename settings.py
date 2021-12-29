'''
trial runs
fps: 5, 15, 25
velocity: 
'''

import math
import numpy as np

CAMERA_SPRAYER_C2C_DIST = 0.5       # m
CAMERA_FPS = 2.4                    # fps
CAMERA_FRAME_CAPTURE_WIDTH = 0.5     # m
CAMERA_CAPTURE_RESOLUTION = 512.     # px

ROBOT_VELOCITY =1.3             # m/s

MAP_ROW_LENGTH = 1000.                # m
MAP_HILL_DIST = 0.2                 # m
MAP_WEED_COVERAGE_RATE = 0.0        # m

class settings():

    def __init__(self):
        
        # robot_velocity
        self.v = ROBOT_VELOCITY     # m/s
        # fps
        self.fps = CAMERA_FPS
        # linear capture resolution
        self.lcr = CAMERA_FRAME_CAPTURE_WIDTH/CAMERA_CAPTURE_RESOLUTION   # m/px
        # frame width
        self.W = CAMERA_FRAME_CAPTURE_WIDTH     # m
        # hill spacing
        self.dh = MAP_HILL_DIST     # m
        # sprayer and camera distance
        self.ds = CAMERA_SPRAYER_C2C_DIST   # m
        # distance travelled per frame
        D = self.v/CAMERA_FPS   # m/frame
        # number of frames
        self.K = math.ceil(MAP_ROW_LENGTH/D)
        # number of hills, round down
        self.nh = int(math.floor(MAP_ROW_LENGTH/MAP_HILL_DIST))
        # number of weeds
        nw = int(math.floor(MAP_WEED_COVERAGE_RATE*self.nh))
        # hill indices, add 1 since np.arange starts at 0
        i = np.arange(self.nh) + 1
        # hill coordinates, 
        self.Xi = (i)*self.dh      # m
        # randomly assign hills as with weeds
        self.ww = np.random.choice(i, nw, replace=False) - 1     # subtract 1 because I added 1 to i

if __name__=="__main__":
    my_settings = settings()
