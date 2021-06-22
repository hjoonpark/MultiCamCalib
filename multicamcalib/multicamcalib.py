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
from helper import load_img_paths, load_config, init_cameras, extract_paths
from corner_detector import detect_corners, generate_detection_results
from outlier_detector import generate_crops_around_corners, train_vae_outlier_detector, run_vae_outlier_detector, determine_outliers
from calibrator import calib_initial_params, estimate_initial_world_points
from analyzer import reproject_world_points, render_reprojection_results


def print_manual():
    print()
    print("#######################################################################################")
    print(":: WELCOME TO Multi-camera Calibration ::")
    print('(example usuage: "python ./multicamcalib.py 1 2 3 4 5 6 7")')
    print()
    print(":: Number menu ::")
    print(">> Corner detection")
    print("  1: run corner detection (multi-threaded)")
    print("  2: generate detection results")
    print(">> VAE outlier corner detection")
    print("  3: generate 15x15 crop images around the detected corners")
    print("  4: train VAE outlier detector")
    print("  5: forward-run VAE outlier detector")
    print("  6: determine outlier corners")
    print(">> Initial calibration")
    print("  7: calibrate initial camera parameters & checkerboard poses")
    print(">> After running bundle adjustment (C++)")
    print("  8: render final camera & checkerboard configurations")
    print("  9: reproject world (checkerboard) points")
    print("  10: analyze reprojection results (render histogram & reprojected images)")
    print("#######################################################################################")
    print()


if __name__ == "__main__":
    # load user-defined config
    config = load_config("config.json")
    paths = extract_paths(config["paths"])

    # initialize logger
    logger = init_logger("MAIN", paths["logs"], init_console_handler=True, init_file_handler=True)
    logger.info("### APPLICATION START ###")

    # python ./multicamcalib.py 1 2 3 
    argv = sys.argv[1:]
    if len(argv) < 1:
        print_manual()
        user_input = input("Type in the numbers(from the menu above) to run delimited by space (ex: 1 2 3 4 5 6 7): ")
        argv = user_input.strip().split(" ")
    
    argv_sorted = sorted([int(x) for x in argv])
    print("Running: ")
    for a in argv_sorted:
        print(">> {}".format(a))


    # load checkerboard configs
    chb_config = config["checkerboard"]
    chb = Checkerboard(chb_config["n_cols"], chb_config["n_rows"], chb_config["sqr_size"])

    # load image paths
    img_paths = load_img_paths(paths["abs_image_paths_file"])

    # initialize cameras from img_paths
    cameras = init_cameras(img_paths)

    if "1" in argv:
        logger.info(">> [1] DETECT CHECKERBOARD CORNERS <<")
        # detect corners
        lock = threading.Lock()
        os.makedirs(paths["corners"], exist_ok=True)
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
        logger.info(">> [2] GENERATE DETECTION RESULTS <<")
        # detection results
        generate_detection_results(logger, cameras, paths)

    if "3" in argv:
        logger.info(">> [3] GENERATE CROPS AROUND CORNERS <<")
        # generate corner crops
        generate_crops_around_corners(logger, img_paths, paths)

    input_crop_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_binary.npy"))))
    vae = config["vae_outlier_detector"]
    vae_config = {"z_dim": vae["z_dim"], "kl_weight": vae["kl_weight"], "lr": vae["lr"], "n_epochs": vae["n_epochs"], "batch_size": vae["batch_size"], "device": "cuda", "debug": False}
    outlier_path = os.path.join(paths["outliers"], "outliers.json")

    if "4" in argv:
        logger.info(">> [4] TRAIN VAE OUTLIER DETECTOR <<")
        # train vae
        train_vae_outlier_detector(logger, input_crop_paths, paths, vae_config)

    if "5" in argv:
        logger.info(">> [5] RUN VAE OUTLIER DETECTOR <<")
        # forward vae
        model_path = os.path.join(paths["vae_outlier_detector"], "vae_model.pt")
        run_vae_outlier_detector(logger, input_crop_paths, paths, model_path, vae_config)

    if "6" in argv:
        logger.info(">> [6] DETERMINE OUTLIERS <<")
        # determine outliers
        save_imgs = True
        model_path = os.path.join(paths["vae_outlier_detector"], "vae_model.pt")
        determine_outliers(logger, vae_config, model_path, input_crop_paths, paths, save_path=outlier_path, outlier_thres_ratio=vae["outlier_thres_ratio"], save_imgs=save_imgs)

    if "7" in argv:
        logger.info(">> [7a] CALIBRATE INITIAL CAMERA PARAMETERS & WORLD POINTS <<")
        # initial camera calibration (PnP)
        calib_done = calib_initial_params(logger, paths, config["calib_initial"], chb, outlier_path=outlier_path)

        if calib_done:
            logger.info(">> [7b] ESTIMATE INITIAL WORLD POINTS <<")
            # initial world points
            estimate_initial_world_points(logger, paths, chb, config)

    if "8" in argv:
        logger.info(">> [8] RENDER FINAL CONFIGURATIONS <<")
        # render final configurations after ceres bundle adjustment
        cam_param_path = os.path.join(paths["cam_params"], "cam_params_final.json")
        world_points_path = os.path.join(paths["world_points"], "world_points_final.json")
        save_path_cam_config = os.path.join(paths["cam_params"], "final_cameras.png")
        save_path_world_points = os.path.join(paths["world_points"], "final_world_points.png")
        center_cam_idx = config["calib_initial"]["center_cam_idx"]
        center_img_name = config["calib_initial"]["center_img_name"]
        render_config(cam_param_path, center_cam_idx, center_img_name, None, "Final cameras", save_path=save_path_cam_config)
        render_config(cam_param_path, center_cam_idx, center_img_name, world_points_path, "Final configuration", save_path=save_path_world_points)
        logger.info("Plots saved:\n\t{}\n\t{}".format(save_path_cam_config, save_path_world_points))

    if "9" in argv:
        logger.info(">> [9] REPROJECT WORLD POINTS <<")
        cam_param_path = os.path.join(paths["cam_params"], "cam_params_final.json")
        world_points_path = os.path.join(paths["world_points"], "world_points_final.json")
        reprojection_save_path = os.path.join(paths["analysis"], "reprojections.json")
        reproject_world_points(logger, cam_param_path, world_points_path, paths, reprojection_save_path=reprojection_save_path)

    if "10" in argv:
        logger.info(">> [10] ANALYZE REPROJECTION RESULTS <<")
        analysis_config = config["analysis"]
        save_reproj_images = analysis_config["save_reproj_images"]
        save_reproj_err_histogram = analysis_config["save_reproj_err_histogram"]
        error_thres = analysis_config["error_thres"]
        render_reprojection_results(logger, paths, save_reproj_err_histogram=save_reproj_err_histogram, save_reproj_images=save_reproj_images, error_thres=error_thres)

    logger.info("* FINISHED RUNNING *")