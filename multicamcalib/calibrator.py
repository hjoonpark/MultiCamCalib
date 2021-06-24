from operator import truediv
from helper import *
from plotter import render_config
import cv2
import glob
import numpy as np
import os
import json
import random

import math as m

def Rx(theta):
    return np.matrix([[ 1, 0           , 0           ],
                    [ 0, m.cos(theta),-m.sin(theta)],
                    [ 0, m.sin(theta), m.cos(theta)]])
def Ry(theta):
    return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                    [ 0           , 1, 0           ],
                    [-m.sin(theta), 0, m.cos(theta)]])

def Rz(theta):
    return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                    [ m.sin(theta), m.cos(theta) , 0 ],
                    [ 0           , 0            , 1 ]])

def calib_initial_params(logger, paths, calib_config, chb, outlier_path=None, save_plot=True):
    random.seed(0)
    
    center_cam_idx = calib_config["center_cam_idx"]
    center_img_name = calib_config["center_img_name"]

    # create output directory
    save_dir = paths["cam_params"]
    os.makedirs(save_dir, exist_ok=True)

    outliers = []
    if outlier_path is not None:
        # load outliers
        with open(outlier_path, "r+") as f:
            j = json.load(f)
            for i, d in j.items():
                outliers.append(d["img_name"])
    outliers = set(outliers)

    corners_dir = paths["corners"]

    # output directory
    cam_param_save_path = os.path.join(save_dir, "cam_params_initial.json")
    cam_params = {}

    cam_folders = [f.path for f in os.scandir(corners_dir) if f.is_dir()]
    cam_indices = sorted([int(s.split("_")[-1]) for s in cam_folders])

    for cam_idx in cam_indices:
        cam_folder = os.path.join(corners_dir, "cam_{}".format(cam_idx))
        cam_params[cam_idx] = {}

        # (2D) load image points randomly
        corner_paths = glob.glob(os.path.join(cam_folder, "*.txt"))
        random.shuffle(corner_paths)
        _2d_pts = []

        imageSize = None
        for corner_path in corner_paths:
            fname = os.path.basename(corner_path).split(".")[0]
            if fname in outliers:
                # skip image that contains outlier corner
                logger.debug("Skipping outlier image for intrinsics: {}".format(fname))
                continue

            corners, imageSizeCurr = load_corner_txt(corner_path) # imageSize: (h, w)

            if corners is not None:
                _2d_pts.append(corners)
                imageSize = imageSizeCurr
                if len(_2d_pts) == calib_config["intrinsics"]["n_max_imgs"]:
                    break
        _2d_pts = np.float32(_2d_pts)

        # (3D) stack chb points
        _3d_pts = np.float32([chb.chb_pts for _ in range(len(_2d_pts))])

        # calibrate intrinsics
        cameraMatrix = None
        distCoeffs = None
        rms_err, M, d, _, _ = cv2.calibrateCamera(_3d_pts, _2d_pts, imageSize=imageSize, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
        logger.info("Intrinsics calibrated: camera {} | {} images | error={:.2f}".format(cam_idx, len(_3d_pts), rms_err))

        # we assume the lens is not fisheye -> let's set lens distortion coefficients to zeros.
        d_scaler = 0.0
        if rms_err:
            cam_params[cam_idx]["fx"] = M[0, 0]
            cam_params[cam_idx]["fy"] = M[1, 1]
            cam_params[cam_idx]["cx"] = M[0, 2]
            cam_params[cam_idx]["cy"] = M[1, 2]
            cam_params[cam_idx]["k1"] = d[0, 0]*d_scaler
            cam_params[cam_idx]["k2"] = d[0, 1]*d_scaler
            cam_params[cam_idx]["p1"] = d[0, 2]*d_scaler
            cam_params[cam_idx]["p2"] = d[0, 3]*d_scaler
            cam_params[cam_idx]["k3"] = d[0, 4]*d_scaler
        else:
            cam_params[cam_idx] = None
            
    # calibrate extrinsics
    # 1. pnp for the center image
    corner_path = os.path.join(corners_dir, "cam_{}".format(center_cam_idx), "{}_{}.txt".format(center_cam_idx, center_img_name))
    _2d_pts, _ = load_corner_txt(corner_path)
    if _2d_pts is None:
        logger.critical("No corner detected for: {}\nProvide a different image name!".format(corner_path))
        assert 0, "No corner detection for the center image! Use a different image."
    
    _3d_pts = chb.chb_pts
    p = cam_params[center_cam_idx]
    M = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
    d = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
    ret, rvec, tvec = cv2.solvePnP(_3d_pts, _2d_pts, M, d)
    cam_params[center_cam_idx]["rvec"] = rvec.flatten().tolist()
    cam_params[center_cam_idx]["tvec"] = tvec.flatten().tolist()

    # 2. stereo calibration between the adjacent cameras
    stereo_transformations = {}

    n_cams = len(cam_folders)
    adj_cam_indices = [i for i in range(center_cam_idx-1, -1, -1)]
    adj_cam_indices.extend([i for i in range(center_cam_idx+1, n_cams)])
    
    for cam_idx_2 in adj_cam_indices:
        corner2_paths = glob.glob(os.path.join(os.path.join(corners_dir, "cam_{}".format(cam_idx_2)), "*.txt"))
        cam_idx_1 = (cam_idx_2 + 1) if cam_idx_2 < center_cam_idx else cam_idx_2 - 1
        stereo_transformations[cam_idx_2] = {}
    
        corners_1 = []
        corners_2 = []
        for corner_path_2 in corner2_paths:
            img_name = os.path.basename(corner_path_2).split(".")[0].split("_")[-1]
            fname_1 = "{}_{}".format(cam_idx_1, img_name)
            fname_2 = "{}_{}".format(cam_idx_2, img_name)
            if (fname_1 in outliers) or (fname_2 in outliers):
                # skip image that contains outlier corner
                logger.debug("Skipping outlier image for extrinsics: {}, {}".format(fname_1, fname_2))
                continue

            corner_path_1 = os.path.join(corners_dir, "cam_{}".format(cam_idx_1), "{}_{}.txt".format(cam_idx_1, img_name))
            
            if not os.path.exists(corner_path_1):
                continue

            pts_1, imageSize = load_corner_txt(corner_path_1)
            if pts_1 is None:
                continue

            pts_2, imageSize = load_corner_txt(corner_path_2)
            if pts_2 is None:
                continue
            
            corners_1.append(pts_1)
            corners_2.append(pts_2)

            if len(corners_1) == calib_config["extrinsics"]["n_max_stereo_imgs"]:
                break
        
        _2d_pts_1 = np.float32(corners_1)
        _2d_pts_2 = np.float32(corners_2)

        _3d_pts = np.float32([chb.chb_pts for _ in range(len(corners_1))])

        flags = 0
        # flags |= cv2.CALIB_FIX_INTRINSIC  # we already have intrinsics (initial values)
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS # optmize intrinsics
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        # flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K1
        # flags |= cv2.CALIB_FIX_K2
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5
        # flags |= cv2.CALIB_FIX_K6

        # termination criteria for the iterative optimization algorithm
        criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 1000, 1e-5)

        # stereo calibrate
        p = cam_params[cam_idx_1]
        M_1 = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
        d_1 = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
        p = cam_params[cam_idx_2]
        M_2 = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
        d_2 = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
        ret, mtx_1, dist_1, mtx_2, dist_2, dR, dt, E, F = cv2.stereoCalibrate(_3d_pts, _2d_pts_1, _2d_pts_2,
                                                                        M_1, d_1, M_2, d_2,
                                                                        imageSize, criteria=criteria, flags=flags)
        logger.info("Stereo-calibrated: camera {} & {} | {} images | error={:.2f}".format(cam_idx_1, cam_idx_2, len(corners_1), ret))

        if ret:
            # dR, dt: brings points given in the first camera's coordinate system to points in the second camera's coordinate system
            # R2 = dR@R1 
            # t2 = dR@t1 + dt 
            stereo_transformations[cam_idx_2]["R"] = dR
            stereo_transformations[cam_idx_2]["t"] = dt.reshape(3, 1)
        else:
            logger.error("Stereo calibration between cam {} and cam {} FAILED!".format(cam_idx_1, cam_idx_2))
            logger.error('  Perhaps try different values for configs["calib_initial"]["extrinsics"]["n_max_stereo_imgs"].')
            logger.error("  Check checkerboard corners are valid for both cameras.")
            assert(0)

    # convert stereo extrinsics to global extrinsics
    R, _ = cv2.Rodrigues(np.float32(cam_params[center_cam_idx]["rvec"]))
    t = np.float32(cam_params[center_cam_idx]["tvec"]).reshape(3, 1)

    for cam_idx in adj_cam_indices:
        assert(cam_idx != center_cam_idx)
        if cam_idx == center_cam_idx - 1 or cam_idx == center_cam_idx + 1:
            R, _ = cv2.Rodrigues(np.float32(cam_params[center_cam_idx]["rvec"]))
            t = np.float32(cam_params[center_cam_idx]["tvec"]).reshape(3, 1)

        dR = stereo_transformations[cam_idx]["R"]
        dt = stereo_transformations[cam_idx]["t"]

        # extrinsics of current camera
        R = dR@R
        t = dR@t + dt

        rvec_2, _ = cv2.Rodrigues(R)
        tvec_2 = t

        cam_params[cam_idx]["rvec"] = rvec_2.flatten().tolist()
        cam_params[cam_idx]["tvec"] = tvec_2.flatten().tolist()
    
    cam_params_sorted = {}
    for cam_idx in sorted(list(cam_params.keys())):
        cam_params_sorted[cam_idx] = cam_params[cam_idx]

    with open(cam_param_save_path, "w+") as f:
        json.dump(cam_params_sorted, f, indent=4)
    logger.info("Initial camera parameters saved: {}".format(cam_param_save_path))

    if save_plot:
        cam_plot_save_path = os.path.join(save_dir, "initial_cameras.png")
        render_config(cam_param_save_path, center_cam_idx, center_img_name, None, "Initial cameras", cam_plot_save_path)
        logger.info("Plot saved: {}".format(cam_plot_save_path))

    return True

def estimate_initial_world_points(logger, paths, chb, config):
    # load detection result
    detect_path = os.path.join(paths["corners"], "detection_result.json")
    with open(detect_path, "r") as f:
        detections = json.load(f)["detections"]

    img_names = sorted(list(detections.keys()))
    center_img_name = config["calib_initial"]["center_img_name"]
    center_cam_idx = config["calib_initial"]["center_cam_idx"]

    # load initial camera parameters
    in_cam_param_path = os.path.join(paths["cam_params"], "cam_params_initial.json")
    with open(in_cam_param_path, "r") as f:
        cam_params = json.load(f)

    cam_indices = sorted([int(i) for i in cam_params.keys()])

    world_pts = {"checkerboard": {"n_rows": chb.n_rows, "n_cols": chb.n_cols, "sqr_size": chb.sqr_size}, "frames": {}}
    for i, img_name in enumerate(img_names):
        rvec_mean = np.float32([0, 0, 0]).reshape(3, )
        tvec_mean = np.float32([0, 0, 0]).reshape(3, )
        ang_mean = 0
        n_cams = 0

        for cam_idx in cam_indices:
            detected = detections[img_name][str(cam_idx)]
            
            if not detected:
                continue

            corner_path = os.path.join(paths["corners"], "cam_{}".format(cam_idx), "{}_{}.txt".format(cam_idx, img_name))
            corners, _ = load_corner_txt(corner_path)
            assert(corners is not None)
            
            R_ext, _ = cv2.Rodrigues(np.float32(cam_params[str(cam_idx)]["rvec"]))
            t_ext = np.float32(cam_params[str(cam_idx)]["tvec"]).reshape(3, 1)

            p = cam_params[str(cam_idx)]
            
            M = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
            d = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
            _, rvec, tvec = cv2.solvePnP(chb.chb_pts, corners, M, d) # brings points from the model frame to the camera frame.

            tvec_norm = np.linalg.norm(tvec)
            if tvec_norm > 1e5:
                logger.debug("Skipping large tvec: {}\timg={}".format(tvec_norm, img_name))
            else:
                R_ext_curr, _ = cv2.Rodrigues(rvec)
                R_model2world = R_ext.T @ R_ext_curr
                t_model2world = R_ext.T @ (tvec - t_ext)

                rvec, _ = cv2.Rodrigues(R_model2world)
                tvec = t_model2world
                ang = np.linalg.norm(rvec)

                if ang > 1e-5:
                    rvec_mean += (rvec.reshape(3, ) / ang)
                    ang_mean += ang
                tvec_mean += tvec.reshape(3, )
                n_cams += 1
                    
        # find average chb pose
        if n_cams > 0:
            tvec_mean /= n_cams
            rvec_mean /= n_cams
            ang_mean /= n_cams

            rvec = rvec_mean * ang_mean
            R, _ = cv2.Rodrigues(rvec)

            # chb_pts: (N, 3), R: (3, 3), tvec: (3, N)
            tvec_mean = tvec_mean.reshape(3, 1)
            tvec = np.repeat(tvec_mean, chb.chb_pts.shape[0], axis=1)
            pts = (R @ chb.chb_pts.T + tvec).T

            world_pts["frames"][img_name] = {"n_detected": n_cams, "rvec": rvec.flatten().tolist(), "tvec": tvec_mean.flatten().tolist(), "world_pts": pts.tolist()}
            # world_pts[img_name] = {"n_detected": n_cams, "rvec": rvec.flatten().tolist(), "tvec": tvec_mean.flatten().tolist()}
        else:
            world_pts["frames"][img_name] = {"n_detected": n_cams, "rvec": -1, "tvec": -1, "world_pts": -1}
            # world_pts[img_name] = {"n_detected": n_cams, "rvec": -1, "tvec": -1}
            logger.debug("[{}/{}] No detection for image {}".format(i+1, len(img_names), img_name))

    # recenter s.t. center img's chb is at R=I_3x3, t=[0, 0, 0]
    center_chb = world_pts["frames"][center_img_name]
    rvec_center = np.float32(center_chb["rvec"])
    R_center, _ = cv2.Rodrigues(rvec_center)
    dR = R_center.T

    tvec_center = np.float32(center_chb["tvec"])
    dt = -R_center.T@tvec_center

    # recenter checkerboards
    for img_name, pose in world_pts["frames"].items():
        if pose["rvec"] == -1:
            continue
        
        R, _ = cv2.Rodrigues(np.float32(pose["rvec"]))
        t = np.float32(pose["tvec"])

        # retarget: H_new = dH @ H
        R_new = dR@R
        tvec_new = dR@t + dt

        rvec_new, _ = cv2.Rodrigues(R_new)
        tvec_new_stacked = np.repeat(tvec_new.reshape(3, 1), chb.chb_pts.shape[0], axis=1)
        pts = (R_new @ chb.chb_pts.T + tvec_new_stacked).T

        world_pts["frames"][img_name]["rvec"] = rvec_new.flatten().tolist()
        world_pts["frames"][img_name]["tvec"] = tvec_new.flatten().tolist()
        world_pts["frames"][img_name]["world_pts"] = pts.tolist()

    # recenter cameras
    for cam_idx, p in cam_params.items():
        R_ext, _ = cv2.Rodrigues(np.float32(p["rvec"]))
        t_ext = np.float32(p["tvec"])

        # retarget: H_new = dH @ H = dH @ H_ext^(-1)
        R_new = dR@R_ext.T
        t_new = -dR@R_ext.T@t_ext + dt

        R_ext_new = R_new.T
        rvec_new, _ = cv2.Rodrigues(R_ext_new)
        tvec_new = -R_new.T@t_new

        cam_params[cam_idx]["rvec"] = rvec_new.flatten().tolist()
        cam_params[cam_idx]["tvec"] = tvec_new.flatten().tolist()

    # re-save initial camera parameters
    in_cam_param_path = os.path.join(paths["cam_params"], "cam_params_initial.json")
    
    with open(in_cam_param_path, "w+") as f:
        json.dump(cam_params, f, indent=4)

    save_dir = paths["world_points"]
    os.makedirs(save_dir, exist_ok=True)

    world_points_save_path = os.path.join(save_dir, "world_points_initial.json")
    with open(world_points_save_path, "w+") as f:
        json.dump(world_pts, f, indent=4)

    logger.info("Initial world points saved: {}".format(world_points_save_path))
    wp_plot_save_path = os.path.join(save_dir, "initial_world_points.png")

    center_cam_idx = config["calib_initial"]["center_cam_idx"]
    center_img_name = config["calib_initial"]["center_img_name"]
    render_config(in_cam_param_path, center_cam_idx, center_img_name, world_points_save_path, "Initial configuration", wp_plot_save_path)
    
    logger.info("Initial world points plot saved: {}".format(wp_plot_save_path))