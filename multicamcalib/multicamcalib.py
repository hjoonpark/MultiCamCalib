from plotter import render_config
from camera import *
from checkerboard import *
from logger import init_logger
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
    # load user-defined config
    config = load_config("config.json")
    paths_dict = config["paths"]
    paths = {}
    for name, path in paths_dict.items():
        if name != "output_dir" and name != "image_paths_file":
            paths[name] = os.path.join(paths_dict["output_dir"], path)
        else:
            paths[name] = path

    # initialize logger
    logger = init_logger("MAIN", paths["logs"], init_console_handler=True, init_file_handler=True)
    logger.info("### APPLICATION START ###")

    # python ./multicamcalib.py 1 2 3 
    argv = sorted(sys.argv[1:])

    # load checkerboard configs
    chb_config = config["checkerboard"]
    chb = Checkerboard(chb_config["n_cols"], chb_config["n_rows"], chb_config["sqr_size"])

    # load image paths
    img_paths = load_img_paths(paths["image_paths_file"])

    # initialize cameras from img_paths
    cameras = init_cameras(img_paths)

    if "1" in argv:
        # detect corners
        lock = threading.Lock()
        shared_log_path = os.path.join(paths["corners"], "cornerdetect_completion.json")
        detect_corners(logger, lock, chb, img_paths, paths, shared_log_path, use_threads=True, log=True)

        # wait until all corner detection threads are complete
        n_complete = 0
        while n_complete != len(cameras):
            n_complete = 0
            lock.acquire()
            with open(shared_log_path, "r") as f:
                completion = json.load(f)
            lock.release()

            for cam_idx, done in completion.items():
                n_complete += int(done)
            time.sleep(1) # check every 1000 ms
    
    if "2" in argv:
        logger.info(">> GENERATE DETECTION RESULTS <<")
        # detection results
        generate_detection_results(logger, cameras, paths)

    if "3" in argv:
        logger.info(">> GENERATE CROPS AROUND CORNERS <<")
        # generate corner crops
        generate_crops_around_corners(logger, img_paths, paths)

    input_crop_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_binary.npy"))))
    vae = config["vae_outlier_detector"]
    vae_config = {"z_dim": vae["z_dim"], "kl_weight": vae["kl_weight"], "lr": vae["lr"], "n_epochs": vae["n_epochs"], "batch_size": vae["batch_size"], "device": "cuda", "debug": False}
    outlier_path = os.path.join(paths["outliers"], "outliers.json")

    if "4" in argv:
        logger.info(">> TRAIN VAE OUTLIER DETECTOR <<")
        # train vae
        train_vae_outlier_detector(logger, input_crop_paths, paths, vae_config)

    if "5" in argv:
        logger.info(">> RUN VAE OUTLIER DETECTOR <<")
        # forward vae
        model_path = os.path.join(paths["vae_outlier_detector"], "vae_model.pt")
        run_vae_outlier_detector(logger, input_crop_paths, paths, model_path, vae_config)

    if "6" in argv:
        logger.info(">> DETERMINE OUTLIERS <<")
        # determine outliers
        determine_outliers(logger, paths, save_path=outlier_path, thres_loss_percent=vae["outlier_thres"], save_imgs=True)

    if "7" in argv:
        logger.info(">> CALIBRATE INITIAL CAMERA PARAMETERS <<")
        # initial camera calibration (PnP)
        calib_initial_params(logger, paths, config["calib_initial"], chb, outlier_path=outlier_path)

    if "8" in argv:
        logger.info(">> ESTIMATE INITIAL WORLD POINTS <<")
        # initial world points
        estimate_initial_world_points(logger, paths, config["calib_initial"], chb)

    if "9" in argv:
        logger.info(">> RENDER FINAL CONFIGURATIONS <<")
        # render final configurations after ceres bundle adjustment
        cam_param_path = os.path.join(paths["cam_params"], "cam_params_final.json")
        world_points_path = os.path.join(paths["world_points"], "world_points_final.json")
        render_config(cam_param_path, None, "Final configuration", save_path=os.path.join(paths["cam_params"], "config_final.png"))
        render_config(cam_param_path, world_points_path, "Final configuration", save_path=os.path.join(paths["world_points"], "final_world_points.png"))

    logger.info("* FINISHED RUNNING *")