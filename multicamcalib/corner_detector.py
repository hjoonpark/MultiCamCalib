from helper import *
import threading
import cv2
import glob
import time
def detect_corners(lock, chb, img_paths, output_dir, shared_log_path, use_threads=True, log=True):
    threads = []
    for cam_idx, paths in img_paths.items():
        if use_threads:
            # log path
            if log:
                log_path = os.path.join(output_dir, "log_cornerdetect_cam{}.txt".format(cam_idx))
            else:
                log_path = None
            
            __write_cornerdet_completion_log(lock, shared_log_path, cam_idx, False)

            # output folder
            output_folder = os.path.join(output_dir, "corners", "cam_{}".format(cam_idx))
            os.makedirs(output_folder, exist_ok=True)
            threads.append(threading.Thread(target=__corner_detector, args=(lock, cam_idx, chb.n_cols, chb.n_rows, paths, output_folder, log_path, shared_log_path)))
    for thread in threads:
        thread.start()
        time.sleep(0.01)

def generate_detection_results(cameras, paths):
    dir_corners = paths["corners"]
    detections_raw = {} # key: img_name, value: dict {cam_idx: 1/0}
    num_detections = {} # key: cam_idx, value: int
    for cam in cameras:
        if cam.idx not in num_detections:
            num_detections[cam.idx] = 0

        corner_dir = os.path.join(dir_corners, "cam_{}".format(cam.idx))
        corner_paths = sorted(glob.glob(os.path.join(corner_dir, "*.txt")))

        for p in corner_paths:
            img_name = os.path.split(p)[-1].split(".")[0].split("_")[-1]
            corners, _ = load_corner_txt(p) # (N, 2) where N is the number of checkerboard corners

            detected = int(corners is not None)
            if img_name not in detections_raw:
                detections_raw[img_name] = {cam.idx: detected}
            else:
                detections_raw[img_name].update({cam.idx: detected})


    # delete frames with less than len(cameras) cameras
    detections = {}
    for img_name in sorted(list(detections_raw.keys())):
        if len(detections_raw[img_name].keys()) == len(cameras):
            detections[img_name] = detections_raw[img_name]

            for cam_idx, detected in detections[img_name].items():
                num_detections[cam_idx] += detected

    result = {
        "num_frames": len(detections.keys()),
        "num_cams": len(cameras),
        "num_detections": num_detections,
        "detections": detections
        }
    save_path = os.path.join(paths["output_dir"], "detection_result.json")
    with open(save_path, 'w+') as f:
        json.dump(result, f, indent=4)
    
    print("  - detection result saved to: {}".format(save_path))

def __write_cornerdet_completion_log(lock, shared_log_path, cam_idx, value):
    lock.acquire()
    if os.path.exists(shared_log_path):
        with open(shared_log_path, "r+") as f:
            shared_log = json.load(f)
    
        shared_log[str(cam_idx)] = value
        shared_log_out = {}
        for i in sorted(map(int, list(shared_log.keys()))):
            shared_log_out[str(i)] = shared_log[str(i)]

        with open(shared_log_path, "w") as f:
            json.dump(shared_log_out, f, indent=4)
    else:
        with open(shared_log_path, "w+") as f:
            json.dump({str(cam_idx): value}, f, indent=4)
    lock.release()

def __corner_detector(lock, cam_idx, n_cols, n_rows, paths, output_folder, log_path, shared_log_path):
    print("Running corner detector: camera {}\t| {} images".format(cam_idx, len(paths)))
    output_log(log_path, "---------- START ----------\n")
    
    time_start = time.time()
    for i, path in enumerate(paths):
        cam_n_img_name = os.path.split(path)[-1].split(".")[0]
        out_path = os.path.join(output_folder, "{}.txt".format(cam_n_img_name))

        # load image
        img_original = load_img(path)
        
        # if image is too large, corner detection takes too long
        scale = 0.5
        h, w = img_original.shape
        if min(h, w) > 1080:
            img = cv2.resize(img_original, dsize=(int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)

        output_log(log_path, "[{}] ({}/{})\tDetecting: {}\t".format(convert_sec(time.time() - time_start), i+1, len(paths)+1, path))

        found, corners = cv2.findChessboardCorners(img, (n_cols, n_rows), cv2.CALIB_CB_FAST_CHECK)

        output_str = []
        if found:
            output_str.append("1 {} {}".format(img_original.shape[0], img_original.shape[1]))
            output_log(log_path, "FOUND\n")
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            # refine corners
            corners /= scale
            corners_refined = cv2.cornerSubPix(img_original, corners, (7, 7), (-1, -1), criteria)

            for uv in corners_refined:
                uv_str = "{} {}".format(uv[0][0], uv[0][1])
                output_str.append("\n{}".format(uv_str))
        else:
            output_str.append("0")
            output_log(log_path, "NOT FOUND\n")

        with open(out_path, "w+") as f:
            f.writelines(output_str)

        del img
    output_log(log_path, "\n* COMPLETE! Saved to: {}\n".format(output_folder))
    __write_cornerdet_completion_log(lock, shared_log_path, cam_idx, True)

    print("* Camera {}\tcomplete! Saved to: {}".format(cam_idx, output_folder))