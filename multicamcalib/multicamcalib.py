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

menu = {
    "1": "corner detection",
    "1a": "detect checkerboard corners (multi-thread)",
    "1b": "generate detection result json",
    "2": "vae outlier corner detection",
    "2a": "generate crop images around detected corners",
    "2b": "train VAE outlier detector",
    "2c": "run VAE outlier detector",
    "2d": "determine outlier corners",
    "3": "initial calibration",
    "3a": "estimate initial camera parameters",
    "3b": "estimate initial checkerboard poses",
    "4": "final calibration (bundle adjustment)",
    "5": "analysis",
    "5a": "render final camera & checkerboard configurations",
    "5b": "reproject world points (render histogram & reprojected images)"
}


def print_manual():
    print()
    print("#######################################################################################")
    print(":: Multi-camera Calibration ::")
    print()
    print(":: Code numbers ::")
    for k, v in menu.items():
        if k.isnumeric():
            print("[{}] {}".format(k, v.upper()))
        else:
            print("   [{}] {}".format(k, v.capitalize()))
    print()
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
        user_input = input("Type in numbers from the menu above, delimited by space (ex: 1 2 3 4 5 <or> 5a 5b 5c): ")
        argv = sorted(user_input.strip().split(" "))
    
    print("Running: ")
    for a in argv:
        print(">> {}".format(a))

    # load checkerboard configs
    chb_config = config["checkerboard"]
    chb = Checkerboard(chb_config["n_cols"], chb_config["n_rows"], chb_config["sqr_size"])

    # load image paths
    img_paths = load_img_paths(paths["abs_image_paths_file"])

    # initialize cameras from img_paths
    cameras = init_cameras(img_paths)

    # some shared variables
    vae = config["vae_outlier_detector"]
    vae_config = {"z_dim": vae["z_dim"], "kl_weight": vae["kl_weight"], "lr": vae["lr"], "n_epochs": vae["n_epochs"], "batch_size": vae["batch_size"], "device": "cuda", "debug": False}
    outlier_path = os.path.join(paths["outliers"], "outliers.json")
    cam_param_path = os.path.join(paths["cam_params"], "cam_params_final.json")
    world_points_path = os.path.join(paths["world_points"], "world_points_final.json")
    save_path_cam_config = os.path.join(paths["cam_params"], "final_cameras.png")
    save_path_world_points = os.path.join(paths["world_points"], "final_world_points.png")
    center_cam_idx = config["calib_initial"]["center_cam_idx"]
    center_img_name = config["calib_initial"]["center_img_name"]

    for code_number in argv:
        assert code_number in menu.keys(), "Code number not in the menu! Check your inputs."

        logger.info(">>>>>>>>>> [{}] {} <<<<<<<<<<".format(code_number, menu[code_number].capitalize()))

        if (code_number == "1a") or (code_number == "1"):
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
        
        if (code_number == "1b") or (code_number == "1"):
            # detection results
            generate_detection_results(logger, cameras, paths)

        if (code_number == "2a") or (code_number == "2"):
            # generate corner crops
            generate_crops_around_corners(logger, img_paths, paths)

        if (code_number == "2b") or (code_number == "2"):
            # train vae
            input_crop_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_binary.npy"))))
            train_vae_outlier_detector(logger, input_crop_paths, paths, vae_config)

        if (code_number == "2c") or (code_number == "2"):
            # forward vae
            input_crop_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_binary.npy"))))
            model_path = os.path.join(paths["vae_outlier_detector"], "vae_model.pt")
            run_vae_outlier_detector(logger, input_crop_paths, paths, model_path, vae_config)

        if (code_number == "2d") or (code_number == "2"):
            # determine outliers
            save_imgs = True
            model_path = os.path.join(paths["vae_outlier_detector"], "vae_model.pt")
            determine_outliers(logger, vae_config, model_path, input_crop_paths, paths, save_path=outlier_path, outlier_thres_ratio=vae["outlier_thres_ratio"], save_imgs=save_imgs)

        if (code_number == "3a") or (code_number == "3"):
            # initial camera calibration (PnP)
            calib_initial_params(logger, paths, config["calib_initial"], chb, outlier_path=outlier_path)

        if (code_number == "3b") or (code_number == "3"):
            # initial world points
            estimate_initial_world_points(logger, paths, chb, config)

        if (code_number == "4"):
            # bundle adjustment (C/C++)
            bundle_adjustment_path = os.path.join("..", "ceres_bundle_adjustment", "build", "bin", "Release", "CeresMulticamCalib.exe")
            assert os.path.exists(bundle_adjustment_path), "CeresMulticamCalib.exe does not exist!"
            os.system(bundle_adjustment_path)

        if (code_number == "5a") or (code_number == "5"):
            # render final configurations after ceres bundle adjustment
            compute_reproj_errs = True
            cam_param_path = os.path.join(paths["cam_params"], "cam_params_final.json")
            world_points_path = os.path.join(paths["world_points"], "world_points_final.json")
            render_config(paths, cam_param_path, center_cam_idx, center_img_name, None, "Final cameras", compute_reproj_errs=compute_reproj_errs, save_path=save_path_cam_config)
            render_config(paths, cam_param_path, center_cam_idx, center_img_name, world_points_path, "Final configuration", compute_reproj_errs=compute_reproj_errs, save_path=save_path_world_points)
            logger.info("Two plots saved:\n\t{}\n\t{}".format(save_path_cam_config, save_path_world_points))

        if (code_number == "5b") or (code_number == "5"):
            reprojection_save_path = os.path.join(paths["analysis"], "reprojections.json")
            is_finished = reproject_world_points(logger, cam_param_path, world_points_path, paths, reprojection_save_path=reprojection_save_path)

            if is_finished:
                analysis_config = config["analysis"]
                save_reproj_images = analysis_config["save_reproj_images"]
                save_reproj_err_histogram = analysis_config["save_reproj_err_histogram"]
                error_thres = analysis_config["error_thres"]
                render_reprojection_results(logger, paths, save_reproj_err_histogram=save_reproj_err_histogram, save_reproj_images=save_reproj_images, error_thres=error_thres)
    
    logger.info("***** FINISHED *****\n")