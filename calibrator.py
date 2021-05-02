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
def calib_initial_params(output_dir, chb_config, calib_config, center_cam_idx, center_img_name, outlier_path=None, save_plot=True):
    random.seed(0)
    # create rest pose checkerboard points
    chb_pts = create_chb_points(n_cols=chb_config["n_cols"], n_rows=chb_config["n_rows"], s=chb_config["sqr_size"])

    # create output directory
    save_dir = os.path.join(output_dir, "cam_params", "initial")
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
    save_path_intrinsics = os.path.join(save_dir, "intrinsics.json")
    save_path_extrinsics = os.path.join(save_dir, "extrinsics.json")
    cam_intrinsics = {}
    cam_extrinsics = {}

    cam_folders = [f.path for f in os.scandir(corners_dir) if f.is_dir()]
    
    for cam_folder in cam_folders:
        cam_idx = int(cam_folder.split("_")[-1])
        cam_intrinsics[cam_idx] = {}
        cam_extrinsics[cam_idx] = {}

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
        _3d_pts = np.float32([chb_pts for _ in range(len(_2d_pts))])

        # calibrate intrinsics
        cameraMatrix = None
        distCoeffs = None
        rms_err, M, d, _, _ = cv2.calibrateCamera(_3d_pts, _2d_pts, imageSize=imageSize, cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)

        if rms_err:
            cam_intrinsics[cam_idx]["fx"] = M[0, 0]
            cam_intrinsics[cam_idx]["fy"] = M[1, 1]
            cam_intrinsics[cam_idx]["cx"] = M[0, 2]
            cam_intrinsics[cam_idx]["cy"] = M[1, 2]
            cam_intrinsics[cam_idx]["k1"] = d[0, 0]
            cam_intrinsics[cam_idx]["k2"] = d[0, 1]
            cam_intrinsics[cam_idx]["p1"] = d[0, 2]
            cam_intrinsics[cam_idx]["p2"] = d[0, 3]
            cam_intrinsics[cam_idx]["k3"] = d[0, 4]
        else:
            cam_intrinsics[cam_idx] = None
            
    cam_intrinsics_sorted = {}
    for cam_idx in sorted(list(cam_intrinsics.keys())):
        cam_intrinsics_sorted[cam_idx] = cam_intrinsics[cam_idx]

    with open(save_path_intrinsics, "w+") as f:
        json.dump(cam_intrinsics_sorted, f, indent=4)
    print("  - Initial intrinsics saved: {}".format(save_path_intrinsics))

    # calibrate extrinsics
    # 1. pnp for the center image
    corner_path = os.path.join(corners_dir, "cam_{}".format(center_cam_idx), "{}_{}.txt".format(center_cam_idx, center_img_name))
    _2d_pts, _ = load_corner_txt(corner_path)
    if _2d_pts is None:
        print("[ERROR] No corner detected for: {}".format(corner_path))
        print("Provide a different image name!")
        assert(0)
    
    _3d_pts = chb_pts
    p = cam_intrinsics[center_cam_idx]
    M = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
    d = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
    ret, rvec, tvec = cv2.solvePnP(_3d_pts, _2d_pts, M, d)
    cam_extrinsics[center_cam_idx]["rvec"] = rvec.flatten().tolist()
    cam_extrinsics[center_cam_idx]["tvec"] = tvec.flatten().tolist()

    # 2. stereo calibration between the center camera and the rest
    stereo_extrinsics = {}
    center_corner_paths = glob.glob(os.path.join(os.path.join(corners_dir, "cam_{}".format(center_cam_idx)), "*.txt"))
    for cam_folder in cam_folders:
        cam_idx = int(cam_folder.split("_")[-1])
        if cam_idx == center_cam_idx:
            continue
        
        stereo_extrinsics[cam_idx] = {}
    
        corners_A = []
        corners_B = []
        for corner_path_A in center_corner_paths:
            img_name = os.path.basename(corner_path_A).split(".")[0].split("_")[-1]
            fname_A = "{}_{}".format(center_cam_idx, img_name)
            fname_B = "{}_{}".format(cam_idx, img_name)
            if (fname_A in outliers) or (fname_B in outliers):
                # skip image that contains outlier corner
                print("\tskipping outlier image for extrinsics: {}, {}".format(fname_A, fname_B))
                continue

            corner_path_B = os.path.join(corners_dir, "cam_{}".format(cam_idx), "{}_{}.txt".format(cam_idx, img_name))
            
            if not os.path.exists(corner_path_B):
                continue

            pts_B, imageSize = load_corner_txt(corner_path_B)
            if pts_B is None:
                continue
            
            pts_A, imageSize = load_corner_txt(corner_path_A)
            if pts_A is None:
                continue

            corners_A.append(pts_A)
            corners_B.append(pts_B)

            if len(corners_A) == calib_config["extrinsics"]["n_max_stereo_imgs"]:
                break
        
        _2d_pts_A = np.float32(corners_A)
        _2d_pts_B = np.float32(corners_B)

        _3d_pts = np.float32([chb_pts for _ in range(len(corners_A))])

        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC  # we already have intrinsics (initial values)
        # flags |= cv2.CALIB_USE_INTRINSIC_GUESS # optmize intrinsics
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        # flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K1
        # flags |= cv2.CALIB_FIX_K2
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5
        # flags |= cv2.CALIB_FIX_K6

        # termination criteria for the iterative optimization algorithm
        criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 1000, 1e-5)

        # stereo calibrate
        p = cam_intrinsics[cam_idx]
        M_B = np.float32([[p["fx"], 0, p["cx"]], [0, p["fy"], p["cy"]], [0, 0, 1]])
        d_B = np.float32([p["k1"], p["k2"], p["p1"], p["p2"], p["k3"]])
        ret, mtx_A, dist_A, mtx_B, dist_B, R_BA, t_BA, E, F = cv2.stereoCalibrate(_3d_pts, _2d_pts_A, _2d_pts_B,
                                                                        M, d, M_B, d_B,
                                                                        imageSize, criteria=criteria, flags=flags)

        if ret:
            stereo_extrinsics[cam_idx]["R"] = R_BA
            stereo_extrinsics[cam_idx]["t"] = t_BA.flatten()
        else:
            print("[ERROR] Stereo calibration between cam {} and cam {} FAILED!".format(center_cam_idx, cam_idx))
            print('\t- Perhaps try different values for configs["calib_initial"]["extrinsics"]["n_max_stereo_imgs"].')
            print("\t- Check checkerboard corners are valid for both cameras.")
            assert(0)

    # convert stereo extrinsics to global extrinsics
    R_wA, _ = cv2.Rodrigues(np.float32(cam_extrinsics[center_cam_idx]["rvec"]))
    t_wA = np.float32(cam_extrinsics[center_cam_idx]["tvec"])
    
    for cam_folder in cam_folders:
        cam_idx = int(cam_folder.split("_")[-1])
        if cam_idx == center_cam_idx:
            continue
        R_BA = stereo_extrinsics[cam_idx]["R"]
        t_BA = stereo_extrinsics[cam_idx]["t"]

        # SE3 of camera w.r.t. global coordinates
        R_AB = R_BA.T
        t_AB = -R_BA.T.dot(t_BA)
        R_SE3 = R_wA.dot(R_AB)
        t_SE3 = R_wA.dot(t_AB.reshape(3,)) + t_wA

        # SE3 to extrinsics
        R_wB = R_SE3.T
        t_wB = -R_SE3.T.dot(t_SE3)

        rvec_B, _ = cv2.Rodrigues(R_wB)
        tvec_B = t_wB

        cam_extrinsics[cam_idx]["rvec"] = rvec_B.tolist()
        cam_extrinsics[cam_idx]["tvec"] = tvec_B.tolist()
    
    cam_extrinsics_sorted = {}
    for cam_idx in sorted(list(cam_extrinsics.keys())):
        cam_extrinsics_sorted[cam_idx] = cam_extrinsics[cam_idx]

    with open(save_path_extrinsics, "w+") as f:
        json.dump(cam_extrinsics_sorted, f, indent=4)
    print("  - Initial extrinsics saved: {}".format(save_path_extrinsics))


    if save_plot:
        # draw 3d plot showing cameras
        save_path = os.path.join(save_dir, "initial_config.png")

        fig = plt.figure(figsize=(10, 8))
        ax = plt.axes(projection='3d')
        L = 1000
        ax.plot([0, L], [0, 0], [0, 0], c="r", linewidth=3)
        ax.plot([0, 0], [0, L], [0, 0], c="g", linewidth=3)
        ax.plot([0, 0], [0, 0], [0, L], c="b", linewidth=3)

        lim_val = -np.inf
        for cam_idx, v in cam_extrinsics_sorted.items():
            tvec = np.float32(v["tvec"])
            rvec = np.float32(v["rvec"])
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
        plt.close()
        print("  - Plot saved: {}".format(save_path))
