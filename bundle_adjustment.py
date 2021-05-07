import os
import json

def generate_bundle_adjustment_input(output_dir):
    # initial camera parameters
    cam_dir = os.path.join(output_dir, "cam_params", "initial")
    with open(os.path.join(cam_dir, "intrinsics.json"), "r") as f:
        cam_params = json.load(f)

    with open(os.path.join(cam_dir, "extrinsics.json"), "r") as f:
        ext = json.load(f)
        for cam_idx, p in ext.items():
            cam_params[p]["rvec"] = p["rvec"]
            cam_params[p]["tvec"] = p["tvec"]

    # initial world points
    # input image points
    