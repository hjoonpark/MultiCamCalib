from camera import *
from checkerboard import *
import json
import threading
import os
import time
import glob
from datetime import datetime
from helper import load_img_paths, load_config, init_cameras
from corner_detector import detect_corners, generate_detection_results
from outlier_detector import generate_crops_around_corners, train_vae_outlier_detector, run_vae_outlier_detector, determine_outliers
from calibrator import calib_initial_params, estimate_initial_world_points
from bundle_adjustment import generate_bundle_adjustment_input

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
    lock = threading.Lock()
    # shared_log_path = os.path.join(config["output_dir"], "cornerdetect_completion.json")
    # detect_corners(lock, chb, img_paths, config["output_dir"], shared_log_path, use_threads=True, log=True)

    # # wait until all corner detection threads are complete
    # n_complete = 0
    # while n_complete != len(cameras):
    #     n_complete = 0
    #     lock.acquire()
    #     with open(shared_log_path, "r") as f:
    #         completion = json.load(f)
    #     lock.release()

    #     for cam_idx, done in completion.items():
    #         n_complete += int(done)
    #     time.sleep(1) # check every 1000 ms


    # detection results
    # generate_detection_results(cameras, config["output_dir"])

    # generate corner crops
    # generate_crops_around_corners(img_paths, config["output_dir"])

    # train vae
    input_crop_paths = sorted(list(glob.glob(os.path.join(config["output_dir"], "corner_crops", "*_binary.npy"))))
    vae = config["vae_outlier_detector"]
    vae_config = {"z_dim": vae["z_dim"], "kl_weight": vae["kl_weight"], "lr": vae["lr"], "n_epochs": vae["n_epochs"], "batch_size": vae["batch_size"], "device": "cuda", "debug": False}
    # train_vae_outlier_detector(input_crop_paths, config["output_dir"], vae_config)

    # forward vae
    model_path = os.path.join(config["output_dir"], "vae_outlier_detector", "vae_model.pt")
    # run_vae_outlier_detector(input_crop_paths, config["output_dir"], model_path, vae_config)

    # determine outliers
    outlier_path = os.path.join(config["output_dir"], "vae_outlier_detector", "outliers.json")
    # determine_outliers(config["output_dir"], save_path=outlier_path, thres_loss_percent=vae["outlier_thres"], save_imgs=True)

    # initial camera calibration (PnP)
    # calib_initial_params(config["output_dir"], config["checkerboard"], config["calib_initial"], outlier_path=outlier_path)

    # initial world points
    estimate_initial_world_points(config["output_dir"], config["checkerboard"], config["calib_initial"])

    # generate inputs for bundle adjustment
    # generate_bundle_adjustment_input(config["output_dir"])