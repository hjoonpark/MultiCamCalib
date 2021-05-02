import os
import glob

in_dir = r"D:\Pictures\2019_12_13_Lada_Capture\Converted"
out_path = "image_paths.txt"


cams = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P']

img_paths = {}
for cam_idx, cam in enumerate(cams):
    in_dir2 = os.path.join(in_dir, cam, "*.pgm")
    paths = sorted(glob.glob(in_dir2))

    for p in paths:
        fnumber = int(os.path.split(p)[-1].split(".")[0].split("_")[-1])
        if fnumber % 5 == 0:
            if cam_idx not in img_paths:
                img_paths[cam_idx] = [p]
            else:
                if len(img_paths[cam_idx]) < 1000000000000000000000000000:
                    img_paths[cam_idx].append(p)
                else:
                    break

with open(out_path, 'w+') as f:
    for cam_idx, paths in img_paths.items():
        for p in paths:
            f.write("{} {}\n".format(cam_idx, p))

print("Done")            
