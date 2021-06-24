
##### Table of Contents  
[Introduction](#s_intro)  
[Installation](#s_overview)  

<h2 id="s_intro">1. Introduction</h2>

---
This project calibrates multiple cameras using a planar calibration checkerboard. The pipeline is comprised of four main steps: [(1)](#step_1) detect checkerboard corners, [(2)](#step_2) remove outlier corners from the previous step (*VAE, variational auto-encoder*), [(3)](#step_3) estimate initial camera parameters and world (checkerboard) points, then finally [(4)](#step_4) refine the initial estimates (*bundle adjustment*).


<figure>
<img src="./docs/readme_assets/bundle_adjustment.gif" width="600px"/>
<figcaption>Bundle adjustment jointly refining the initial parameters of 16 cameras (black) and world points (green).</figcaption>
</figure>

* This project is focused on being simple and scalable, applicable to different calibration checkerboards and different number of cameras.
* The steps (1)-(3) are implemented using *python 3* and the step (4) using *C/C++*.
* On the example dataset provided, this project achieves *mean reprojection error of 0.08 pixels* with standard deviation 0.05.

<h2 id="s_overview">2. Overview</h2>
The pipeline is as followed:

<img src="./docs/readme_assets/pipeline.jpg" alt="pipeline" style="max-width:1000px;width:100%"/>

*(Input)* (0). **Multi-view images**: The input is a set of images capturing a freely moving checkerboard.

<label id="step_1">(1)</label>. **Corner detection**: Checkerboard corners are detected and localized with sub-pixel precision using [OpenCV](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html).
(Run: 1, 2)


<label id="step_2">(2)</label>. **VAE outlier detector**: Outlier corners are identified using VAE (variational auto-encoder), and corresponding images are discarded.
(Run: 3, 4, 5, 6)

<label id="step_3">(3)</label>. **Initial camera calibration**: Initial camera parameters and frame-wise checkerboard poses are estimated from a subset of images. 
(Run: 7)

<label id="step_4">(4)</label>. **Bundle adjustment**: The camera parameters and frame-wise checkerboard poses are further refined using bundle adjustment.
(Run: CeresMulticamCalib.exe)

*(Optional)* (5). Analyze the calibration result. (Run: 8, 9, 10)

<h2 id="s_installation">3. Installation</h2>

---

(1) Download the project

    git clone https://github.com/hjoonpark/MultiCamCalib.git

(2) 

### Ceres installation
#### 1. [Windows](docs/install_windows.md)
#### 2. [Linux](docs/install_linux.md) (coming)


---
<h2 id="s_example">Example (Step-by-step tutorial)</h2>

The best way to get familiarized is by running an example.

Click [HERE](docs/tutorial.md) for a step-by-step tutoral.

<h2 id="s_details">4. Academical details</h2>

----
[Link](docs/details.md)
---
