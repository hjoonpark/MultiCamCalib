
##### Table of Contents  
[Introduction](#s_intro)  
[Installation](#s_overview)  

<h2 id="s_intro">1. Introduction</h2>

---
This project calibrates multiple cameras using a planar calibration checkerboard. The pipeline is comprised of *four* main steps: [(1)](#step_1) detect checkerboard corners, [(2)](#step_2) remove outlier corners from the previous step (using *VAE, variational auto-encoder*), [(3)](#step_3) estimate initial camera parameters and world (checkerboard) points, [(4)](#step_4) refine the initial estimates (using *bundle adjustment*), then *optionally* [(5)](#step_5) analyze the calibration result.


<figure style="display:inline-block; display:block;">
<img src="./docs/assets/tutorial/studio.jpg" width="67%"/>
<img src="./docs/assets/bundle_adjustment_v2.gif" width="30%"/>
<figcaption> Left: 16 synchronized cameras-setup. Right: Bundle adjustment jointly refining the initial parameters of 16 cameras (black) and world points (green).</figcaption>
</figure>
</br>

* This project is focused on being simple and scalable, applicable to different calibration checkerboards and different number of cameras.
* The steps (1)-(3)/(5) are implemented using *python 3* and the step (4) using *C/C++*.
* On the example dataset provided (./example_data/), this project achieves *mean reprojection error 0.08 pixels* with standard deviation 0.05.

<h2 id="s_overview">2. Overview</h2>

The code execution follows the pipeline below:

<img src="./docs/assets/pipeline.jpg" alt="pipeline" style="max-width:1000px;width:100%"/>

*(Input)* (0). **Multi-view images**: The input is a set of images capturing a freely moving checkerboard.

<label id="step_1">(1)</label>. **Corner detection**: Checkerboard corners are detected and localized with sub-pixel precision using [OpenCV](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html).

<label id="step_2">(2)</label>. **VAE outlier detector**: Outlier corners are identified using VAE (variational auto-encoder), and corresponding images are discarded. As expained [here](./docs/details.md#s_vae), open source corner detectors (e.g., OpenCV, MATLAB, etc.) are known to give incorrect or inaccurate (outlier) corners which may deteriorate the accuracy of camera caibration. Therefore, we identify and remove images containing the outlier corners.

<label id="step_3">(3)</label>. **Initial camera calibration**: Initial camera parameters and frame-wise checkerboard poses are first estimated from a subset of images.

<label id="step_4">(4)</label>. **Bundle adjustment**: The estimated camera parameters and frame-wise checkerboard poses are further refined using bundle adjustment.

*(Optional)* <label id="step_5">(5)</label>. **Analyze the calibration result**: Computes reprojection errors and render corresponding histogram, reprojected images, etc. for analysis.

<h2 id="s_installation">3. Installation</h2>

---

(1) Download the project

    git clone https://github.com/hjoonpark/MultiCamCalib.git

(2) Install Ceres
* Click [here](docs/install_windows.md) for **Windows**
* Click here for **Linux** (coming)

(3) Configure & compile CeresMultiCamCalib.exe

(4) Install [Anaconda](https://docs.anaconda.com/anaconda/install/windows/) environment

(5) Run the codes! → Follow [this tutorial](#s_example).


<h2 id="s_example">4. Tutorial</h2>

---

Follow [this quick start tutorial (a step-by-step example)](docs/tutorial.md)

<h2 id="s_details">5. Academical details</h2>

---

Click [here](docs/details.md).

<h2 id="s_contact">6. Contact</h2>

---

For questions, please contact hjoonpark.us@gmail.com.