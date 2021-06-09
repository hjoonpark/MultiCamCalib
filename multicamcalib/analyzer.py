import numpy as np
import json
import cv2
import os
import math
from tqdm import tqdm
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from helper import load_corner_txt

def _reproject(param, world_pts):
    # world_pts: (N, 3)
    R = param["R"]
    t = param["t"]
    fx = param["fx"]
    fy = param["fy"]
    cx = param["cx"]
    cy = param["cy"]
    k1 = param["k1"]
    k2 = param["k2"]
    p1 = param["p1"]
    p2 = param["p2"]
    k3 = param["k3"]

    # world to camera frame
    x = np.dot(world_pts, R.T) + t

    # camera to image frame
    x = x[:, 0:2] / x[:, 2][:, None]


    # lens distortions following opencv lens model
    r2 = x[:, 0]**2 + x[:,1]**2
    radial = 1 + k1*r2 + k2*r2**2 + k3*r2**3
    tan_u = (p2 * (r2 + 2*x[:,0]**2) + 2*p1*x[:,0]*x[:,1])
    tan_v = (p1 * (r2 + 2*x[:,1]**2) + 2*p2*x[:,0]*x[:,1])
    u = x[:,0]*radial + tan_u
    v = x[:,1]*radial + tan_v
    
    # following pinhole camera model
    u = fx*u + cx
    v = fy*v + cy
    
    return np.vstack([u, v]).T
    


def analyze_calibration_result(logger, cam_param_path, world_points_path, paths, outliers_path=None, save_histogram=True, save_images=True):
    os.makedirs(paths["analysis"], exist_ok=True)

    # load camera parameters
    with open(cam_param_path, "r") as f:
        cam_params = json.load(f)
        for cam_idx, param in cam_params.items():
            rvec = np.float32(param["rvec"])
            R, _ = cv2.Rodrigues(rvec)
            t = np.float32(param["tvec"])
            cam_params[cam_idx]["R"] = R
            cam_params[cam_idx]["t"] = t

    # load world points to reproject
    with open(world_points_path, "r") as f:
        world_points = json.load(f)["frames"]
    
    # load outliers
    if outliers_path is not None:
        with open(outliers_path, "r") as f:
            outliers_json = json.load(f)
            outliers = []
            for _, v in outliers_json.items():
                outliers.append(v["img_name"])
            outliers = set(outliers)
    else:
        outliers = []

    # reproject each image points
    reprojections = {} # img_name: {cam_idx: {img_pts: [], errors: []}}
    corner_errors = []
    corner_errors_2d = []
    errors_for_lookup = {} # img_name: max_error
    pbar = tqdm(total=len(world_points.keys()))
    for img_name, d in world_points.items():
        pbar.update(1)

        world_pts = np.float32(d["world_pts"])
        reprojections[img_name] = {}
        errors_for_lookup[img_name] = -1
        for cam_idx in cam_params.keys():
            corner_path = os.path.join(paths["corners"], "cam_{}".format(cam_idx), "{}_{}.txt".format(cam_idx, img_name))
            img_pts, _ = load_corner_txt(corner_path)

            if img_pts is None:
                continue

            reprojections[img_name][cam_idx] = {}
            img_pts_pred = _reproject(cam_params[cam_idx], world_pts)
            dudv = img_pts - img_pts_pred
            err_each = np.sqrt(np.sum(dudv**2, axis=1))
            err_sum = float(np.sum(err_each))
            corner_errors.extend(err_each)
            corner_errors_2d.extend(dudv)
            reprojections[img_name][cam_idx] = {"error_sum": err_sum, "img_pts_pred": img_pts_pred.tolist()}

            errors_for_lookup[img_name] = max(errors_for_lookup[img_name], err_sum)

    # save reprojections as json
    json_save_path = os.path.join(paths["analysis"], "reprojections.json")
    with open(json_save_path, "w+") as f:
        json.dump(reprojections, f, indent=4)
        logger.info("Reprojections saved: {}".format(json_save_path))

    # plot historgrams
    corner_errors = np.float32(corner_errors)
    corner_errors_2d = np.float32(corner_errors_2d)
    fs = 16
    fig = plt.figure(figsize=(16, 5))
    gs = GridSpec(1, 10, figure=fig)
    ax1 = fig.add_subplot(gs[0, 0:6])
    ax2 = fig.add_subplot(gs[0, 7:])

    bins_scale = min(3, int(np.log10(len(corner_errors))))
    ax1.hist(corner_errors, bins=10**bins_scale)
    ax1.set_xlim(left=0)
    ax1.set_yscale('log')
    ax1.set_xlabel("Reprojection error [pixel]", fontsize=fs)
    ax1.set_ylabel("Bin count (log-scale)", fontsize=fs)
    ax1.set_title("Reprojection errors of every checkerboard points", fontsize=fs)
    
    ax2.scatter(corner_errors_2d[:,0], corner_errors_2d[:,1], s=0.5)
    ax2.set_xlabel("du [pixel]", fontsize=fs)
    ax2.set_ylabel("dv [pixel]", fontsize=fs)
    ax2.set_title("Reprojection errors in 2D", fontsize=fs)

    max_err = np.max(corner_errors_2d)
    min_err = np.min(corner_errors_2d)
    ax2.set_xlim([min_err, max_err])
    ax2.set_ylim([min_err, max_err])

    plt.suptitle("Camera parameters: {}\nWorld points: {}".format(cam_param_path, world_points_path), fontsize=fs/2.5)
    if save_histogram:
        plot_save_path = os.path.join(paths["analysis"], "reproj_err_histograms.png")
        plt.savefig(plot_save_path, dpi=150)
        plt.close()
        logger.info("Plot saved: {}".format(plot_save_path))
    else:
        plt.show()

    # save images that has large reprojection errors
    if save_images:
        # save images that has reprojection errors larger than {err_thres}
        err_thres = 5

        for img_name, err in errors_for_lookup.items():
            if err > err_thres:
                N_sqrt = np.sqrt(len(cam_params.keys()))
                c = math.floor(N_sqrt)
                if N_sqrt > c**2:
                    r = c+1
                else:
                    r = c

                fig, ax = plt.subplots(r, c, figsize=(20, 20))
                ax = ax.ravel()

                for cam_idx in cam_params.keys():
                    a = ax[int(cam_idx)]
                    if cam_idx in reprojections:
                        pred = np.float32(reprojections[cam_idx]["img_pts_pred"])
                        a.scatter(pred[:,0], pred[:,1], c="b", s=1)
                    else:
                        a.grid(False)
                plt.show()
            break

                    
        


