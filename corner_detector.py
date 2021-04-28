from helper import *
import threading
import cv2
import glob
def detect_corners(chb, img_paths, output_dir, use_threads=True, log=True):
    threads = []
    for cam_idx, paths in img_paths.items():
        if use_threads:
            # log path
            if log:
                log_path = os.path.join(output_dir, "log_cornerdetect_cam{}.txt".format(cam_idx))
            else:
                log_path = None
            
            # output folder
            output_folder = os.path.join(output_dir, "corners", "cam_{}".format(cam_idx))
            os.makedirs(output_folder, exist_ok=True)
            threads.append(threading.Thread(target=__corner_detector, args=(cam_idx, chb.n_cols, chb.n_rows, paths, output_folder, log_path)))

    for thread in threads:
        thread.start()

def generate_detection_results(cameras, output_dir):
    detections = {} # key: img_name, value: dict {cam_idx: 1/0}
    num_detections = {} # key: cam_idx, value: int
    for cam in cameras:
        corner_dir = os.path.join(output_dir, "corners", "cam_{}".format(cam.idx))
        corner_paths = sorted(glob.glob(os.path.join(corner_dir, "*.txt")))

        for p in corner_paths:
            img_name = os.path.split(p)[-1].split(".")[0].split("_")[-1]
            corners = load_corner_txt(p) # (N, 2) where N is the number of checkerboard corners

            detected = int(corners is not None)
            if img_name not in detections:
                detections[img_name] = {cam.idx: detected}
            else:
                detections[img_name].update({cam.idx: detected})

            if cam.idx not in num_detections:
                num_detections[cam.idx] = 0
            
            num_detections[cam.idx] += detected

    result = {
        "num_frames": len(detections.keys()),
        "num_cams": len(cameras),
        "num_detections": num_detections,
        "detections": detections
        }
    save_path = os.path.join(output_dir, "detection_result.json")
    with open(save_path, 'w+') as f:
        json.dump(result, f, indent=4)
    
    print(" * Detection result saved: {}".format(save_path))


def __corner_detector(cam_idx, n_cols, n_rows, paths, output_folder, log_path):
    print("Running corner detector: camera {}\t| {} images".format(cam_idx, len(paths)))
    output_log(log_path, "---------- START ----------\n")
    
    for i, path in enumerate(paths):
        cam_n_img_name = os.path.split(path)[-1].split(".")[0]
        out_path = os.path.join(output_folder, "{}.txt".format(cam_n_img_name))

        # load image
        img = load_img(path)
        
        output_log(log_path, "  - detecting: {}\n".format(path))
        ret, corners = cv2.findChessboardCorners(img, (n_cols, n_rows), cv2.CALIB_CB_FAST_CHECK)

        output_str = []
        if ret:
            output_str.append("1")
            output_log(log_path, "    - FOUND\n")
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(img, corners, (7, 7), (-1, -1), criteria)

            for uv in corners_refined:
                uv_str = "{} {}".format(uv[0][0], uv[0][1])
                output_str.append("\n{}".format(uv_str))

        else:
            output_str.append("0")
            output_log(log_path, "    - NOT FOUND\n")

        with open(out_path, "w+") as f:
            f.writelines(output_str)

        del img
    output_log(log_path, "\n* COMPLETE! Saved to: {}\n".format(output_folder))
    print("* Camera {}\tcomplete! Saved to: {}".format(cam_idx, output_folder))