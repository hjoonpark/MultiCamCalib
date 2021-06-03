import os
import json
import cv2
from camera import *

def load_img_paths(path_file):
    img_paths = {}
    with open(path_file, 'r') as f:
            ls = f.readlines()
            for l in ls:
                vs = l.split(" ")
                cam_idx = int(vs[0])
                img_path = vs[1].split("\n")[0]
                if cam_idx not in img_paths:
                    img_paths[cam_idx] = [img_path]
                else:
                    img_paths[cam_idx].append(img_path)
    return img_paths

def load_img(path):
    img = cv2.imread(path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img

def init_cameras(img_paths):
    cameras = []
    for cam_idx in sorted(list(img_paths.keys())):
        cameras.append(Camera(cam_idx))
    return cameras

def load_config(path):
    with open(path, 'r') as f:
        return json.load(f)

def load_corner_txt(path):
    with open(path, 'r') as f:
        lines = f.readlines()
        first_line = lines[0].split(" ")
        is_detected = int(first_line[0])
        if is_detected == 1:
            # chb detected
            img_height = int(first_line[1])
            img_width = int(first_line[2])

            corners = []
            for i in range(1, len(lines)):
                line = lines[i].split(" ")
                corners.append([float(line[0]), float(line[1])])
            return np.float32(corners), (img_height, img_width)
        else:
            return None, None

def convert_sec(seconds): 
    min, sec = divmod(seconds, 60) 
    hour, min = divmod(min, 60) 
    return "%d:%02d:%02d" % (hour, min, sec) 

# def output_log(path, strs):
#     if path is not None:
#         mode = "a+" if os.path.exists(path) else "w+"
#         with open(path, mode) as f:
#             f.write(strs)
#             f.close()