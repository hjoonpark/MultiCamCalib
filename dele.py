# import os
# import glob
# cams = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P']
# in_dir = r"D:\Pictures\2019_12_13_Lada_Capture\Converted"
# for i, cam in enumerate(cams):
#     in_path = glob.glob(os.path.join(in_dir, cam, "*.pgm"))
#     for path in in_path:
#         fname = os.path.split(path)[-1].split(".")[0][1:]
#         out_path = os.path.join(in_dir, cam, "{}_{}.pgm".format(i, fname))
#         print(path)
#         print(out_path)
#         os.rename(path, out_path)
#         print()