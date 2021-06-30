<h1>Academical details</h1>

<h2>Table of contents</h2>

---

1. [Camera model](#camera-model)
2. [Bundle adjustment with planar checkerboard](#bundle-adjustment)
3. [VAE outlier corner detection](#s_vae)  
    - [Objective of VAE](#vae-objective)
    - [VAE architecture](#vae-architecture)
4. [Experimental results](#experimental)
    - [VAE outlier corner detector](#exp-vae)
        - [Effects of corner outliers on the calibration accuracy](#vae-effects)
        - [Performance evaluations](#vae-performance)
            - [Synthetic experiment](#performance-synth)
            - [Real experiment](#performance-real)
    - [Improvements on 3D reconstructions accuracy](#exp-recon)

- [References](#references)

<h2 id="camera-model">1. Camera model</h2>

---

We model the cameras as the usual pinhole camera following Zhang's method<sup>[[1]](#zhang2000flexible)</sup>. A 3D world point is denoted as ![$p=[x, y, z]$](./assets/eq/world_point.jpg) and its projected image point as ![$q=[u, v]$](./assets/eq/image_point.jpg), related by:

![pinhole-model](./assets/eq/camera_model.jpg),

where ![$(R, t)$](./assets/eq/rt.jpg) are extrinsic parameters with rotation (parameterized as the angle-axis vector) and translation, and ![s](./assets/eq/s.jpg) is an arbitrary scale factor. ![K](./assets/eq/k.jpg) is the camera intrinsic matrix with focal lengths ![$(fx, fy)$](./assets/eq/fxfy.jpg) and principal points ![$(cx, cy)$](./assets/eq/cxcy.jpg) in pixel unit. ![$\gamma$](./assets/eq/gamma.jpg) describes the skew of the two image axes which can be assumed zero for most modern cameras<sup>[[2]](#szeliski2010computer)</sup>. 

The lens distortions are modeled using the coefficients ![$(k1, k2, k3)$](./assets/eq/k1k2k3.jpg) for radial and ![$(p1, p2)$](./assets/eq/p1p2.jpg) for tangential distortions following the widely used lens model<sup>[[2]](#szeliski2010computer)</sup> relating a distorted image point ![$q_d$](./assets/eq/q_d.jpg) and undistorted image point ![$q$](./assets/eq/q.jpg) by:

![lens-model](./assets/eq/lens_distortion.jpg)

As a result, each camera is modeled with 15 parameters
(6 extrinsics, 4 intrinsics, and 5 lens distortion parameters).

<h2 id="bundle-adjustment">2. Bundle adjustment with planar checkboard</h2>

---
We denote the positions of checkerboard corners w.r.t. the body coordinate frame as a vectorized form ![$\bb{p}^b=[x_1, y_1, z_1, \dots, x_K, y_K, z_K]^T\in\mathbb{R}^{3K}$](./assets/eq/chb_pts.jpg) where ![$K$](./assets/eq/kk.jpg) is the number of corners on a checkerboard. To enforce the rigid planar assumption, we make ![$\bb{p}^b$](./assets/eq/pb.jpg) constant and set ![$z_k=0$](./assets/eq/z_k.jpg). Then, a rigid body transformation ![$\mathcal{Q}(\cdot)$](./assets/eq/QQ.jpg) is applied to each corner to obtain their world positions ![$\bb{p}^w$](./assets/eq/pw.jpg). The transformation is parameterized by the angle-axis vector ![$\bm{\omega}$](./assets/eq/omega.jpg) for rotation and translation vector ![$\bm{\tau}$](./assets/eq/tau.jpg):

![chb_pts](./assets/eq/chb_pts2.jpg),

where ![$\mathcal{R}(\cdot)$](./assets/eq/R.jpg) computes a rotation matrix using Rodrigues' formula, and ![$\bb{p}_k^b$](./assets/eq/pbk.jpg) denotes position of ![$k^\text{th}$](./assets/eq/kth.jpg) corner in ![$\bb{p}^b$](./assets/eq/pb.jpg).

Using this planar model, we minimize the following sum of mean reprojection errors over all frames:

![bundle_adjustment](./assets/eq/bundle_adjustment.jpg)

where ![$i$](./assets/eq/i.jpg) is frame index, ![$j$](./assets/eq/j.jpg) is camera index with ![$V_i$](./assets/eq/Vi.jpg) encoding visibility information, ![$\bb{a}_j$](./assets/eq/aj.jpg) is a vector of 15 parameters of camera ![$j$](./assets/eq/j.jpg), ![$(\bm{\bb{\omega}}_i, \bm{\tau}_i)$](./assets/eq/omegatau.jpg) is the pose of the checkerboard, ![$\hat{\bb{q}}_{ij}\in \mathbb{R}^{3K}$](./assets/eq/qhat.jpg) are the vectorized positions of observed corners, ![$\mathcal{P}(\cdot)$](./assets/eq/PP.jpg) projects each of the ![$K$](./assets/eq/kk.jpg) world points ![$\mathcal{Q}(\cdot)$](./assets/eq/QQ.jpg) onto camera's image plane, and ![$\mathcal{N}$](./assets/eq/N.jpg) is for averaging.

<h2 id="s_vae">3. VAE outlier corner detection</h2>

---

OpenCV/Matlab corner detectors are known to give incorrect or inaccurate corners especially when the checkerboard is tilted at a large angle w.r.t. the image plane, is under poor lighting, or other objects are present in the image <sup>[[6](#shu2003automatic), [7](#albarelli2009robust), [8](#wang2010recognition)]</sup>. Therefore, We treat incorrectly or inaccurately detected corners as
outliers and identify them using VAE.

<img src="./assets/corner_example.jpg" width="500px">

<h3 id="vae-objective">Objective of general VAE</h3>

VAE deals with an unknown underlying probabilistic distribution ![$p^*(\bb{x})$](./assets/eq/vae/p_star.jpg) defined over the data points ![$\bb{x}$](./assets/eq/vae/x.jpg) in some potentially high-dimensional space ![$\mathcal{X}$](./assets/eq/vae/xx.jpg)<sup>[[3]](#kingma2013auto)</sup>. The aim of VAE is to approximate the underlying distribution with a chosen model ![$p_{\bb{\theta}}(\bb{x}) \approx p^*(\bb{x})$](./assets/eq/vae/p_theta_approx.jpg), parameterized by ![$\bb{\theta}$](./assets/eq/vae/theta.jpg) and marginalized over the latent variables ![$\bb{z}$](./assets/eq/vae/z.jpg):

![$\bb{x}\sim p_{\bb{\theta}}(\bb{x}) = \int p_{\bb{\theta}}(\bb{x}, \bb{z}) d\bb{z} = \int p_{\bb{\theta}}(\bb{x}|\bb{z}) p_{\bb{\theta}}(\bb{z})d\bb{z}$](./assets/eq/vae/x_approx.jpg).

The objective is to estimate the parameters ![$\bb{\theta}$](./assets/eq/vae/theta.jpg) via Maximum Likelihood Estimation (MLE) on the likelihood of the data set ![$p_{\bb{\theta}}(\bb{X})$](./assets/eq/vae/p_theta_xx.jpg) consisting of i.i.d. samples ![$\bb{X} = \{ \bb{x}_i \}_{i=1}^N$](./assets/eq/vae/x_set.jpg). Using variational inference for approximating the intractable true posterior ![$p_{\theta}(\bb{z}|\bb{x}_i)$](./assets/eq/vae/p_theta_zx.jpg) involved in ![$p_{\theta}(\bb{x}_i)$](./assets/eq/vae/p_theta_xi.jpg) with some tractable distribution ![$q_{\phi}(\bb{z}|\bb{x}_i)$](./assets/eq/vae/q_theta_zx.jpg), then it is sufficient for MLE to maximize the evidence lower bound ![$\mathcal{L}_{\bb{\theta}, \bb{\phi}}(\bb{x}_i)$](./assets/eq/vae/L.jpg):

![lower_bound](./assets/eq/vae/lower_bound.jpg),

where ![$D_{KL}(\cdot)$](./assets/eq/vae/dkl.jpg) is Kullback-Leibler (KL) divergence<sup>[[4]](#murphy2012machine)</sup>.

To make the maximization of the evidence lower bound tractable, the prior ![$p_{\bb{\theta}}(\bb{z})$](./assets/eq/vae/p_theta_z.jpg) is set as isotropic unit Gaussian and the posterior ![$q_{\bb{\phi}}(\bb{z}|\bb{x}_i)$](./assets/eq/vae/q_phi_zxi.jpg) as Gaussians ![$\mathcal{N}(\bm{\mu}_q, \sigma_q^2\bm{I})$](./assets/eq/vae/N_dist.jpg). Also, this parameterization allows the use of the *reparameterization trick* to make the random sampling of ![$\bb{z}$$\sim$$q_{\bb{\phi}}(\bb{z}|\bb{x}_i)$](./assets/eq/vae/z_approx.jpg) in ![$\mathcal{L}_{\bb{\theta}, \bb{\phi}}(\bb{x}_i)$](./assets/eq/vae/L.jpg) differentiable w.r.t. ![$\bb{\phi}$](./assets/eq/vae/phi.jpg) by reparameterizing it as ![$\bb{z} = \bm{\mu}_q + \sigma_q \bm{\epsilon}$](./assets/eq/vae/z_equal.jpg), with ![$\bm{\epsilon}$$\sim$$\mathcal{N}(\bb{0}, \bb{I})$](./assets/eq/vae/epsilon_approx.jpg). Moreover, by choosing ![$p_{\bb{\theta}}(\bb{x}_i|\bb{z})$$\sim$$\mathcal{N}(\bm{\mu}_p, \sigma_p^2 \bm{I})$](./assets/eq/vae/p_theta_xz_approx.jpg) where ![$\sigma_p$](./assets/eq/vae/sigma_p.jpg) is pre-determined and after collecting constant terms into ![$\omega$](./assets/eq/vae/omega.jpg), maximizing ![the right-hand side of Equation \ref{eq:lower-bound}](./assets/eq/vae/L.jpg) becomes equivalent to minimizing:

![vae_obj](./assets/eq/vae/vae_obj.jpg)

For VAEs, ![$q_\phi(\bb{z}|\bb{x})$](./assets/eq/vae/q_phi_zx.jpg) refers to a probabilistic *encoder* and ![$p_\theta(\bb{x}|\bb{z})$](./assets/eq/vae/p_theta_xz.jpg) to a probabilistic *decoder*.


<h3 id="vae-architecture">VAE architecture</h3>

<img src="./assets/eq/vae/vae_architecture.jpg" width="600px">

For input images, we first remove shading information through binarization using Otsu's method following Gaussian blurring. The last layer in the decoder is a sigmoid activation which returns pixel values in range [0,1].

<h2 id="experimental">4. Experimental results</h2>

---

<h3 id="exp-vae">a. VAE outlier corner detector</h3>

1. <label id="vae-effects">Effects of corner outliers on the calibration accuracy</label>

    <figure>
    <img src="./assets/studio_setup.jpg" width="450px" id="studio-setup">
    <figcaption>Figure 1: (a) 16 camera studio setup and (b) its virtual counterpart with a randomly moving checkerboard visualized.</figcaption>
    </figure>

    We generate four sets of synthetic image points of a virtual planar checkerboard ([Figure 1](#studio-setup)). We include different percentages of outlier corners (0%, 0.01%, 0.1%, and 1%) in each image set where each outlier corner is randomly offset from its ground-truth location by 10-15 pixels. Then, we estimate the parameters of 16 cameras using each of the four image sets and compare the root mean square error (RMSE) between the ground-truth.

    <figure>
        <img src="./assets/cam_param_compare_outlier.jpg" width="600px" id="rmse">
        <figcaption>Figure 2: RMSE comparisons of estimated camera parameters, grouped by percentages of outlier corners.</figcaption>
    </figure>

    The result is obtained as in [Figure 2](#rmse), where the geodesic distance on unit sphere is used for computing the error in camera orientations ![$\bb{R}'$](./assets/eq/exp/R_dash.jpg), and ![$\bm{t}'$](./assets/eq/exp/t_dash.jpg) denotes camera position. Since each calibration is obtained in an arbitrary coordinates system, we align them with the reference coordinates using Procrustes analysis before computing RMSE.

2. <label id="vae-performance">Performance evaluations</label>

    *<h4 id="performance-synth">Synthetic experiment</h4>*

    We generate 10k synthetic corner images with 1% outliers by cropping 15x15 pixels from the template corner after applying random affine transformations:

    <img src="./assets/corner_template.jpg" width="120px">

    The outlier corners are offset from the center of crop by 2-4 pixels ([Figure 3b](#synth-crops)).

    We set the latent space of VAE and AE to 2 dimensional, and VAE's KL divergence weight to ![$\omega$](./assets/eq/vae/omega.jpg)=1. Then, VAE and AE are trained for 1k epochs using Adam optimizer with learning rate 1e-3. For PCA and kPCA (with a Gaussian kernel), we use 2 principal axes for reconstructions.

    We use the same binarized inputs for both training and testing ([Figure 3](#synth-crops)) and compare the results using the area under the receiver operator characteristic curve (AUROC) and the precision recall curve (AUPRC). Note, when the data for binary classification is heavily imbalanced, AUPRC can be less misleading than AUROC<sup>[[5]](#davis2006relationship)</sup>.

    <figure>
        <img src="./assets/vae_synth_crops.jpg" width="600px" id="synth-crops">
        <figcaption>Figure 3: Reconstructions and normalized losses of synthetic corners: (a) inliers and (b) outliers.</figcaption>
    </figure>

    From [Figure 4](#auroc-auprc), we observe AUPRC of VAE is the largest, outperforming the rest. For VAE, the precision ratio remains very close to 1 even as the recall ratio approaches 1, implying VAE can remove outliers without also removing the inliers.

    <figure>
        <img src="./assets/auroc_auprc.jpg" width="600px" id="auroc-auprc">
        <figcaption>Figure 4: Area under the receiver operator characteristic curve (AUROC) and the precision recall curve (AUPRC).</figcaption>
    </figure>

    The performance differences are more pronounced in the normalized reconstruction losses plot of every data points in [Figure 5](#recon-loss). In contrast to AE, PCA, and kPCA, a clear classification margin is achieved by VAE which implies the outliers can be identified with high confidence.

    <figure>
        <img src="./assets/recon_loss.jpg" width="600px" id="recon-loss">
        <figcaption>Figure 5: Normalized reconstruction losses of every corner crops by VAE, AE, PCA, and kPCA.</figcaption>
    </figure>

    *<h4 id="performance-real">Real experiment</h4>*

    We capture images (~172 frames per camera) of a freely moving checkerboard and crop 15x15 pixels centered at detected corners. Similarly, we train VAE with heuristically determined KL divergence weight ![$\omega$](./assets/eq/vae/omega.jpg)=0.01 for 200 epochs.

    <figure>
        <img src="./assets/recons_real.jpg" width="600px" id="recons-real">
        <figcaption>Figure 6: Original grayscale crops (above) and VAE reconstructions (below) sorted by normalized losses.</figcaption>
    </figure>

    The result in [Figure 6](#recons-real) shows that the outlier corners are reconstructed with large loss, and via visual inspection 37 outliers can be identified (~ 0.015% outliers). Similar to [Figure 5](#recon-loss), a clear classification margin can be observed in the reconstruction loss plot in [Figure 7](#recon-loss-real). Utilizing these, we can determine the outlier corners with high confidence from tens to thousands of images.

    <figure>
        <img src="./assets/recon_loss_real.jpg" width="600px" id="recon-loss-real">
        <figcaption>Figure 7: Normalized VAE reconstruction losses of every corner crops displaying distinct classification margin.</figcaption>
    </figure>

b. <label id="exp-recon">Improvements on 3D reconstructions accuracy</label>

<figure>
    <img src="./assets/bowlingballs.jpg" width="600px" id="ballingballs">
    <figcaption>Figure 8: (a) Bowling ball captured at three different locations in three frames and (b) its stereo triangulations visualized.</figcaption>
</figure>

We capture images (~172 frames per camera) of a freely moving checkerboard and calibrate cameras using two different methods: (1) with and (2) without outlier corners. Then, we capture images of a polyester bowling ball at three different locations and reconstruct its 34 manually-marked visible feature points ([Figure 8a](#ballingballs)). We triangulate its feature points using pairs of calibrated cameras {(1, 2), (2, 3), ..., (16,1)} to obtain a set of sparse point clouds ([Figure 8b](#ballingballs)). The radius of the ball is measured ~108 mm, but we do not rely solely on this since it is human-measured and the reconstruction error is usually very small (<1 mm). Instead, we evaluate the reconstruction consistency based on the radius values between the triangulated points and center of spheres fitted to each point cloud set via least-square fitting. If the calibrations are accurate, every radius values should have very small deviations from their mean.

<figure>
    <img src="./assets/exp_real.jpg" width="400px" id="exp-real">
    <figcaption>Figure 9: Total of 102 feature points triangulated using camera pairs calibrated with/without corner outliers (<i>mean</i>, <i>standard deviation</i>).</figcaption>
</figure>

The box plots of the obtained radiuses are shown in [Figure 9](#exp-real). The radiuses display smaller deviations with statistically significant difference (*p* < 0.05 from paired sample t-test) when images containing the outlier corners are removed.


<h2 id="references">References</h2>

---
<a id="zhang2000flexible">[1]</a> Zhang Z. A flexible new technique for camera calibration. IEEE Transactions on pattern analysis and machine intelligence. 2000 Nov;22(11):1330-4.

<a id="szeliski2010computer">[2]</a> Szeliski R. Computer vision: algorithms and applications. Springer Science & Business Media; 2010 Sep 30.

<a id="kingma2013auto">[3]</a> Kingma DP, Welling M. Auto-encoding variational bayes. arXiv preprint arXiv:1312.6114. 2013 Dec 20.

<a id="murphy2012machine">[4]</a> Murphy KP. Machine learning: a probabilistic perspective. MIT press; 2012 Sep 7.

<a id="davis2006relationship">[5]</a> Davis J, Goadrich M. The relationship between Precision-Recall and ROC curves. InProceedings of the 23rd international conference on Machine learning 2006 Jun 25 (pp. 233-240).

<a id="shu2003automatic">[6]</a> Shu C, Brunton A, Fiala M. Automatic grid finding in calibration patterns using Delaunay triangulation. National Research Council of Canada; 2003 Aug.

<a id="albarelli2009robust">[7]</a> Albarelli A, Rodolà E, Torsello A. Robust camera calibration using inaccurate targets. IEEE Trans. Pattern Anal. Mach. Intell.. 2009;31:376-83.

<a id="wang2010recognition">[8]</a> Wang Z, Wang Z, Wu Y. Recognition of corners of planar checkboard calibration pattern image. In2010 Chinese Control and Decision Conference 2010 May 26 (pp. 3224-3228). IEEE.