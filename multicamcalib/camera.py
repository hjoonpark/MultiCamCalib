import numpy as np
import cv2

class Camera:
    def __init__(self, cam_idx):
        self.idx = int(cam_idx)

        # extrinsic parameters: maps world space to camera space
        self.rvec = np.float32([0, 0, 0])
        self.tvec = np.float32([0, 0, 0])

        # intrinsic parameters: follows pinhole camera model
        self.fx = 0.0 # focal length x
        self.fy = 0.0 # focal length y
        self.cx = 0.0 # principal point x
        self.cy = 0.0 # principal point y

        # lens distortions: follows Zhang's model
        self.k = [0, 0, 0] # radial distortion: k1, k2, k3
        self.p = [0, 0] # tangential distortion: p1, p2

        # image resolutions in pixels
        self.img_res = (0, 0)
        
        # images captured by this camera
        self.images = []



