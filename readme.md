
##### Table of Contents  
[Introduction](#h1)  
[Installation](#emphasis)  

<h2 id="h1">1. Introduction</h2>

---
This project calibrates multiple cameras using *bundle adjustment* employing a planar calibration checkerboard. After detecting checkerboard corners, we use *VAE outlier corner detector* to remove images with inaccurate or outlier corners which further improves the calibration accuracy.


The steps are as followed.

<img src="./docs/readme_assets/pipeline.jpg" alt="pipeline" style="max-width:1000px;width:100%"/>

(1). **Multi-view images**: The input is a set of images capturing a freely moving checkerboard.

(2). **Corner detection**: Checkerboard corners are detected and localized with sub-pixel precision using [OpenCV](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html).

(3). **VAE outlier detector**: Outlier corners are identified using VAE (variational auto-encoder), and corresponding images are discarded.

(4). **Initial camera calibration**: Initial camera parameters and frame-wise checkerboard poses are estimated from a subset of images.

(5). **Bundle adjustment**: The camera parameters and frame-wise checkerboard poses are further refined using bundle adjustment.

<h2 id="h2">2. Visual demonstration</h2>

---

<h3>Bundle adjustment</h3>

<figure>
<img src="./docs/readme_assets/bundle_adjustment.gif" width="600px"/>
<figcaption>Bundle adjustment for calibrating 16 cameras (black) and checkerboard poses (green).</figcaption>
</figure>

<h2 id="h3">3. Installation</h2>

---

(1) Download the project

    git clone https://github.com/hjoonpark/MultiCamCalib.git

(2) 

### Ceres installation
#### 1. [Windows](docs/install_windows.md)
#### 2. [Linux](docs/install_linux.md) (coming)


<h2 id="h4">4. Academical details</h2>

----
[Link](docs/details.md)
---
