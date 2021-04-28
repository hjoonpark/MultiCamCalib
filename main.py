from camera import *
from checkerboard import *
import json
import threading
import os
import glob
from datetime import datetime
from helper import load_img_paths, load_config, init_cameras
from corner_detector import detect_corners, generate_detection_results
from outlier_detector import generate_crops_around_corners, train_vae_outlier_detector, run_vae_outlier_detector

if __name__ == "__main__":
    config = load_config("config.json")
    os.makedirs(config["output_dir"], exist_ok=True)

    # load checkerboard configs
    chb_config = config["checkerboard"]
    chb = Checkerboard(chb_config["n_cols"], chb_config["n_rows"], chb_config["sqr_size"])

    # load image paths
    img_paths = load_img_paths(config["image_paths_file"])

    # initialize cameras from img_paths
    cameras = init_cameras(img_paths)

    print("==================")
    print(" load image paths")
    print("==================")
    for cam_idx, path in img_paths.items():
        print("  - camera {}:\t{} images".format(cam_idx, len(path)))

    # detect corners
    # detect_corners(chb, img_paths, config["output_dir"], use_threads=True, log=True)

    # detection results
    # generate_detection_results(cameras, config["output_dir"])

    # generate corner crops
    # generate_crops_around_corners(img_paths, config["output_dir"])

    # train vae
    input_crop_paths = sorted(list(glob.glob(os.path.join(config["output_dir"], "corner_crops", "*_binary.npy"))))
    vae_configs = {"device": "cuda", "lr": 1e-3, "n_epochs": 1000, "batch_size": 100, "z_dim": 2, "kl_weight": 0.01, "debug": False}
    # train_vae_outlier_detector(input_crop_paths, config["output_dir"], vae_configs)

    # forward vae
    model_path = os.path.join(config["output_dir"], "vae_outlier_detector", "vae_model.pt")
    run_vae_outlier_detector(input_crop_paths, config["output_dir"], model_path, vae_configs)

    # initial calibration (PnP)
