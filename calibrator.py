from helper import *
import cv2
import glob
import numpy as np
import os
import json
import random
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pickle

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
def draw_camera(ax, cam_idx, rvec, tvec, color="k"):
    w = 1000
    h = w/2
    rect0 = np.float32([[-w/2, -h/2, 0], [-w/2, h/2, 0], [w/2, h/2, 0], [w/2, -h/2, 0], [0, 0, -w*2/3]])
    R, _ = cv2.Rodrigues(rvec)
    R_SE3 = R.T
    t_SE3 = -R.T.dot(tvec)
    
    rect = []
    for r in rect0:
        rect.append(R_SE3@r + t_SE3)
    rect = np.float32(rect)
    ax.plot([rect[0, 0], rect[1,0]], [rect[0, 1], rect[1,1]], [rect[0, 2], rect[1,2]], c=color)
    ax.plot([rect[1, 0], rect[2,0]], [rect[1, 1], rect[2,1]], [rect[1, 2], rect[2,2]], c=color)
    ax.plot([rect[2, 0], rect[3,0]], [rect[2, 1], rect[3,1]], [rect[2, 2], rect[3,2]], c=color)
    ax.plot([rect[3, 0], rect[0,0]], [rect[3, 1], rect[0,1]], [rect[3, 2], rect[0,2]], c=color)

    ax.plot([rect[0, 0], rect[4,0]], [rect[0, 1], rect[4,1]], [rect[0, 2], rect[4,2]], c=color)
    ax.plot([rect[1, 0], rect[4,0]], [rect[1, 1], rect[4,1]], [rect[1, 2], rect[4,2]], c=color)
    ax.plot([rect[2, 0], rect[4,0]], [rect[2, 1], rect[4,1]], [rect[2, 2], rect[4,2]], c=color)
    ax.plot([rect[3, 0], rect[4,0]], [rect[3, 1], rect[4,1]], [rect[3, 2], rect[4,2]], c=color)

    L = 1000
    x = R_SE3@np.float32([L, 0, 0]) + t_SE3
    y = R_SE3@np.float32([0, L, 0]) + t_SE3
    z = R_SE3@np.float32([0, 0, L]) + t_SE3
    ax.plot([t_SE3[0], x[0]], [t_SE3[1], x[1]], [t_SE3[2], x[2]], c="r")
    ax.plot([t_SE3[0], y[0]], [t_SE3[1], y[1]], [t_SE3[2], y[2]], c="g")
    ax.plot([t_SE3[0], z[0]], [t_SE3[1], z[1]], [t_SE3[2], z[2]], c="b")

    # ax.plot([t_SE3[0], t_SE3[0]], [t_SE3[1], t_SE3[1]], [0, t_SE3[2]], linestyle=":", linewidth=1, c="k")

    ax.text(t_SE3[0], t_SE3[1], t_SE3[2]+L, cam_idx, fontsize=12)

def calib_initial_params(output_dir, chb_config, calib_config, chb, outlier_path=None, save_plot=True):
    random.seed(0)
    
    center_cam_idx = calib_config["center_cam_idx"]
    center_img_name = calib_config["center_img_name"]

    # create output directory
    save_dir = os.path.join(output_dir, "cam_params")
    os.makedirs(save_dir, exist_ok=True)

    outliers = []
    if outlier_path is not None:
        # load outliers
        with open(outlier_path, "r+") as f:
            j = json.load(f)
            for i, d in j.items():
                outliers.append(d["img_name"])
    outliers = set(outliers)

    corners_dir = os.path.join(output_dir, "corners")

    # output directory
    save_path = os.path.join(save_dir, "cam_params_initial.json")
    cam_params = {}

    cam_folders = [f.path for f in os.scandir(corners_dir) if f.is_dir()]
    
    for cam_folder in cam_folders:
        cam_idx = int(cam_folder.split("_")[-1])
        cam_params[cam_idx] = {}

        # (2D) load image points randomly
        corner_paths = glob.glob(os.path.join(cam_folder, "*.txt"))
        random.shuffle(corner_paths)
        _2d_pts = []

        for corner_path in corner_paths:
            fname = os.path.basename(corner_path).split(".")[0]
            if fname in outliers:
                # skip image that contains outlier corner
                print("\tskipping outlier image for intrinsics: {}".format(fname))
                continue

            corners, imageSize = load_corner_txt(corner_path) # imageSize: (h, w)
            if corners is not None:
                _2d_pts.append(corners)
                if len(_2d_pts) == calib_config["intrinsics"]["n_max_imgs"]:
                    break
        _2d_pts = np.float32(_2d_pts)

        # (3D) stack chb points
        _3d_pts = np.float32([chb.chb_pts for _ in range(len(_2d_pts))])

        # calibrate intrinsics
        cameraMatrix = None
        distCoeffs = None
        rms_err, M, d, _, _ = cv2.calibrateCamera(_3d_pts, _2d_pts, imageSize=imageSize, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)

        if rms_err:
            cam_params[cam_idx]["fx"] = M[0, 0]
            cam_params[cam_idx]["fy"] = M[1, 1]
            cam_params[cam_idx]["cx"] = M[0, 2]
            cam_params[cam_idx]["cy"] = M[1, 2]
            cam_params[cam_idx]["k1"] = d[0, 0]
            cam_params[cam_idx]["k2"] = d[0, 1]
            cam_params[cam_idx]["p1"] = d[0, 2]
            cam_params[cam_idx]["p2"] = d[0, 3]
            cam_params[cam_idx]["k3"] = d[0, 4]
        else:
            cam_params[cam_idx] = None
            
    # calibrate extrinsics
    # 1. pnp for the center image
    corner_path = os.path.join(corners_dir, "cam_{}".format(center_cam_idx), "{}_{}.txt".format(center_cam_idx, center_img_name))
    _2d_pts, _ = load_corner_txt(corner_path)
    if _2d_pts is None:
        print("[ERROR] No corner detected for: {}".format(corner_path))
        print("Provide a different image name!")
        assert(0)
    
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
    
        print("Stereo calibration: camera {} & {}".format(cam_idx_1, cam_idx_2))
        corners_1 = []
        corners_2 = []
        for corner_path_2 in corner2_paths:
            img_name = os.path.basename(corner_path_2).split(".")[0].split("_")[-1]
            fname_1 = "{}_{}".format(cam_idx_1, img_name)
            fname_2 = "{}_{}".format(cam_idx_2, img_name)
            if (fname_1 in outliers) or (fname_2 in outliers):
                # skip image that contains outlier corner
                print("\tskipping outlier image for extrinsics: {}, {}".format(fname_1, fname_2))
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
        flags |= cv2.CALIB_FIX_INTRINSIC  # we already have intrinsics (initial values)
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

        if ret:
            # dR, dt: brings points given in the first camera's coordinate system to points in the second camera's coordinate system
            # R2 = dR@R1 
            # t2 = dR@t1 + dt 
            stereo_transformations[cam_idx_2]["R"] = dR
            stereo_transformations[cam_idx_2]["t"] = dt.reshape(3, 1)
        else:
            print("[ERROR] Stereo calibration between cam {} and cam {} FAILED!".format(cam_idx_1, cam_idx_2))
            print('\t- Perhaps try different values for configs["calib_initial"]["extrinsics"]["n_max_stereo_imgs"].')
            print("\t- Check checkerboard corners are valid for both cameras.")
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

    with open(save_path, "w+") as f:
        json.dump(cam_params_sorted, f, indent=4)
    print("  - Initial camera parameters saved: {}".format(save_path))

    if save_plot:
        # draw 3d plot showing cameras
        save_path = os.path.join(save_dir, "config_initial.png")

        fig = plt.figure(figsize=(10, 8))
        ax = plt.axes(projection='3d')
        L = 1000
        ax.plot([0, L], [0, 0], [0, 0], c="r", linewidth=3)
        ax.plot([0, 0], [0, L], [0, 0], c="g", linewidth=3)
        ax.plot([0, 0], [0, 0], [0, L], c="b", linewidth=3)

        lim_val = -np.inf
        for cam_idx, v in cam_params_sorted.items():
            tvec = np.float32(v["tvec"]).flatten()
            rvec = np.float32(v["rvec"]).flatten()
            if cam_idx == center_cam_idx:
                c = "r"
            else:
                c = "k"
            draw_camera(ax, cam_idx, rvec, tvec, color=c)

            lim_val = max(lim_val, abs(tvec[0]))
            lim_val = max(lim_val, abs(tvec[1]))
            lim_val = max(lim_val, abs(tvec[2]))
        ax.set_xlim([-lim_val, lim_val])
        ax.set_ylim([-lim_val, lim_val])
        ax.set_zlim([-lim_val, lim_val])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.title("Initial camera configuration")
        plt.savefig(save_path, dpi=150)
        # plt.show()
        plt.close()
        print("  - Plot saved: {}".format(save_path))

def estimate_initial_world_points(output_dir, chb_config, calib_config, chb):
    center_cam_idx = calib_config["center_cam_idx"]

    # load detection result
    detect_path = os.path.join(output_dir, "detection_result.json")
    with open(detect_path, "r") as f:
        detections = json.load(f)["detections"]

    img_names = sorted(list(detections.keys()))

    # load initial camera parameters
    in_path = os.path.join(output_dir, "cam_params", "cam_params_initial.json")
    with open(in_path, "r") as f:
        cam_params = json.load(f)

    cam_indices = sorted([int(i) for i in cam_params.keys()])

    world_pts = {}
    for i, img_name in enumerate(img_names):
        rvec_mean = np.float32([0, 0, 0]).reshape(3, )
        tvec_mean = np.float32([0, 0, 0]).reshape(3, )
        ang_mean = 0
        n_cams = 0

        for cam_idx in cam_indices:
            detected = detections[img_name][str(cam_idx)]
            
            if not detected:
                continue

            corner_path = os.path.join(output_dir, "corners", "cam_{}".format(cam_idx), "{}_{}.txt".format(cam_idx, img_name))
            corners, _ = load_corner_txt(corner_path)
            assert(corners is not None)
            
            R_ext, _ = cv2.Rodrigues(np.float32(cam_params[str(cam_idx)]["rvec"]))
            t_ext = np.float32(cam_params[str(cam_idx)]["tvec"]).reshape(3, 1)

            p = cam_params[str(cam_idx)]
            
            M = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
            d = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
            ret, rvec, tvec = cv2.solvePnP(chb.chb_pts, corners, M, d) # brings points from the model coordinate system to the camera coordinate system.

            tvec_norm = np.linalg.norm(tvec)
            if tvec_norm > 1e5:
                print("skipping large tvec: {}\timg={}".format(tvec_norm, img_name))
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
            world_pts[img_name] = {"n_detected": n_cams, "rvec": rvec.flatten().tolist(), "tvec": tvec_mean.flatten().tolist(), "world_pts": pts.tolist()}
            # world_pts[img_name] = {"n_detected": n_cams, "rvec": rvec.flatten().tolist(), "tvec": tvec_mean.flatten().tolist()}
        else:
            world_pts[img_name] = {"n_detected": n_cams, "rvec": -1, "tvec": -1, "world_pts": -1}
            # world_pts[img_name] = {"n_detected": n_cams, "rvec": -1, "tvec": -1}
            print("  - [{}/{}]\tNo detection for image {}".format(i+1, len(img_names), img_name))

    save_dir = os.path.join(output_dir, "world_points")
    os.makedirs(save_dir, exist_ok=True)

    save_path = os.path.join(save_dir, "world_points_initial.json")
    with open(save_path, "w+") as f:
        json.dump(world_pts, f, indent=4)

    fig = plt.figure(figsize=(10, 8))
    ax = plt.axes(projection='3d')
    L = 1000
    ax.plot([0, L], [0, 0], [0, 0], c="r", linewidth=4)
    ax.plot([0, 0], [0, L], [0, 0], c="g", linewidth=4)
    ax.plot([0, 0], [0, 0], [0, L], c="b", linewidth=4)

    k = 0
    # render initial checkerboard points (world points)
    for img_name, d in world_pts.items():
        if d["n_detected"] > 0:
            p = np.float32(d["world_pts"])
            ax.scatter(p[:,0], p[:,1], p[:,2], c='lime', s=0.1)
            k += 1
    
    # render initial camera configurations
    for cam_idx, v in cam_params.items():
        tvec = np.float32(v["tvec"]).flatten()
        rvec = np.float32(v["rvec"]).flatten()
        if int(cam_idx) == center_cam_idx:
            c = "r"
        else:
            c = "k"
        draw_camera(ax, cam_idx, rvec, tvec, color=c)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title("Initial configuration")
    save_path = os.path.join(save_dir, "intial_world_points.png")
    plt.savefig(save_path, dpi=150)
    plt.close()
    print("  - Initial world points saved to: {}".format(save_path))
    print("  - Plot saved: {}".format(save_path))
