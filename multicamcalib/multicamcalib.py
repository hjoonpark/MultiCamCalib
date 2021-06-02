from camera import *
from checkerboard import *
from logger import *
import json
import threading
import os
import sys
import time
import glob
from datetime import datetime
from helper import load_img_paths, load_config, init_cameras
from corner_detector import detect_corners, generate_detection_results
from outlier_detector import generate_crops_around_corners, train_vae_outlier_detector, run_vae_outlier_detector, determine_outliers
from calibrator import calib_initial_params, estimate_initial_world_points

if __name__ == "__main__":
    # python ./multicamcalib.py 1 2 3 
    argv = sorted(sys.argv[1:])

    config = load_config("config.json")
    paths = config["paths"]

    # initialize logger
    logger = get_logger("multi_cam_calib", paths)

    # load checkerboard configs
    chb_config = config["checkerboard"]
    chb = Checkerboard(chb_config["n_cols"], chb_config["n_rows"], chb_config["sqr_size"])

    # load image paths
    img_paths = load_img_paths(paths["image_paths_file"])

    # initialize cameras from img_paths
    cameras = init_cameras(img_paths)

    logger.info("info")
    logger.debug("debug")
    logger.warning("warning")
    logger.error("error")
    logger.critical("critical")
    
    print("==================")
    print(" load image paths")
    print("==================")
    for cam_idx, path in img_paths.items():
        print("  - camera {}:\t{} images".format(cam_idx, len(path)))

    if "1" in argv:
        # detect corners
        lock = threading.Lock()
        # shared_log_path = os.path.join(paths["output_dir"], "cornerdetect_completion.json")
        # detect_corners(lock, chb, img_paths, paths["output_dir"], shared_log_path, use_threads=True, log=True)

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
    
    if "2" in argv:
        print(">> GENERATE DETECTION RESULTS <<")
        # detection results
        generate_detection_results(cameras, paths)

    if "3" in argv:
        print(">> GENERATE CROPS AROUND CORNERS <<")
        # generate corner crops
        generate_crops_around_corners(img_paths, paths)


    input_crop_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_binary.npy"))))
    vae = config["vae_outlier_detector"]
    vae_config = {"z_dim": vae["z_dim"], "kl_weight": vae["kl_weight"], "lr": vae["lr"], "n_epochs": vae["n_epochs"], "batch_size": vae["batch_size"], "device": "cuda", "debug": False}
    outlier_path = os.path.join(paths["outliers"], "outliers.json")

    if "4" in argv:
        print(">> TRAIN VAE OUTLIER DETECTOR <<")
        # train vae
        train_vae_outlier_detector(input_crop_paths, paths, vae_config)

    if "5" in argv:
        print(">> RUN VAE OUTLIER DETECTOR <<")
        # forward vae
        model_path = os.path.join(paths["vae_outlier_detector"], "vae_model.pt")
        run_vae_outlier_detector(input_crop_paths, paths, model_path, vae_config)

    if "6" in argv:
        print(">> DETERMINE OUTLIERS <<")
        # determine outliers
        determine_outliers(paths, save_path=outlier_path, thres_loss_percent=vae["outlier_thres"], save_imgs=True)

    if "7" in argv:
        print(">> CALIBRATE INITIAL CAMERA PARAMETERS <<")
        # initial camera calibration (PnP)
        calib_initial_params(paths, config["calib_initial"], chb, outlier_path=outlier_path)

    if "8" in argv:
        print(">> ESTIMATE INITIAL WORLD POINTS <<")
        # initial world points
        estimate_initial_world_points(paths, config["calib_initial"], chb)
