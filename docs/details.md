<h1>Academical details</h1>

- [Camera model](#camera-model)
- [Bundle adjustment with planar checkerboard](#bundle-adjustment)
- [VAE outlier corner detection](#vae)  
    - [Objective of VAE](#vae-objective)
    - [VAE architecture](#vae-architecture)
- [Experimental results](#experimental)
    - [Effects of corner outliers on the calibration accuracy](#vae-effects)
    - [Performance evaluations](#vae-performance)
        - [Synthetic experiment](#performance-synth)
        - [Real experiment](#performance-real)
 
- [References](#references)

<h2 id="camera-model">Camera model</h2>

---

We model the cameras as the usual pinhole camera. A 3D world point is denoted as ![$p=[x, y, z]$](./readme_assets/eq/world_point.jpg) and its projected image point as ![$q=[u, v]$](./readme_assets/eq/image_point.jpg), related by:

![pinhole-model](./readme_assets/eq/camera_model.jpg),

where ![$(R, t)$](./readme_assets/eq/rt.jpg) are extrinsic parameters with rotation (parameterized as the angle-axis vector) and translation, and ![s](./readme_assets/eq/s.jpg) is an arbitrary scale factor. ![K](./readme_assets/eq/k.jpg) is the camera intrinsic matrix with focal lengths ![$(fx, fy)$](./readme_assets/eq/fxfy.jpg) and principal points ![$(cx, cy)$](./readme_assets/eq/cxcy.jpg) in pixel unit. ![$\gamma$](./readme_assets/eq/gamma.jpg) describes the skew of the two image axes which can be assumed zero for most modern cameras<sup>[[1]](#szeliski2010computer)</sup>. 

The lens distortions are modeled using the coefficients ![$(k1, k2, k3)$](./readme_assets/eq/k1k2k3.jpg) for radial and ![$(p1, p2)$](./readme_assets/eq/p1p2.jpg) for tangential distortions following the widely used lens model<sup>[[1]](#szeliski2010computer)</sup> relating a distorted image point ![$q_d$](./readme_assets/eq/q_d.jpg) and undistorted image point ![$q$](./readme_assets/eq/q.jpg) by:

![lens-model](./readme_assets/eq/lens_distortion.jpg)

As a result, each camera is modeled with 15 parameters
(6 extrinsics, 4 intrinsics, and 5 lens distortion parameters).

<h2 id="bundle-adjustment">Bundle adjustment with planar checkboard</h2>

---
We denote the positions of checkerboard corners w.r.t. the body coordinate frame as a vectorized form ![$\bb{p}^b=[x_1, y_1, z_1, \dots, x_K, y_K, z_K]^T\in\mathbb{R}^{3K}$](./readme_assets/eq/chb_pts.jpg) where ![$K$](./readme_assets/eq/kk.jpg) is the number of corners on a checkerboard. To enforce the rigid planar assumption, we make ![$\bb{p}^b$](./readme_assets/eq/pb.jpg) constant and set ![$z_k=0$](./readme_assets/eq/z_k.jpg). Then, a rigid body transformation ![$\mathcal{Q}(\cdot)$](./readme_assets/eq/QQ.jpg) is applied to each corner to obtain their world positions ![$\bb{p}^w$](./readme_assets/eq/pw.jpg). The transformation is parameterized by the angle-axis vector ![$\bm{\omega}$](./readme_assets/eq/omega.jpg) for rotation and translation vector ![$\bm{\tau}$](./readme_assets/eq/tau.jpg):

![chb_pts](./readme_assets/eq/chb_pts2.jpg),

where ![$\mathcal{R}(\cdot)$](./readme_assets/eq/R.jpg) computes a rotation matrix using Rodrigues' formula, and ![$\bb{p}_k^b$](./readme_assets/eq/pbk.jpg) denotes position of ![$k^\text{th}$](./readme_assets/eq/kth.jpg) corner in ![$\bb{p}^b$](./readme_assets/eq/pb.jpg).

Using this planar model, we minimize the following sum of mean reprojection errors over all frames:

![bundle_adjustment](./readme_assets/eq/bundle_adjustment.jpg)

where ![$i$](./readme_assets/eq/i.jpg) is frame index, ![$j$](./readme_assets/eq/j.jpg) is camera index with ![$V_i$](./readme_assets/eq/Vi.jpg) encoding visibility information, ![$\bb{a}_j$](./readme_assets/eq/aj.jpg) is a vector of 15 parameters of camera ![$j$](./readme_assets/eq/j.jpg), ![$(\bm{\bb{\omega}}_i, \bm{\tau}_i)$](./readme_assets/eq/omegatau.jpg) is the pose of the checkerboard, ![$\hat{\bb{q}}_{ij}\in \mathbb{R}^{3K}$](./readme_assets/eq/qhat.jpg) are the vectorized positions of observed corners, ![$\mathcal{P}(\cdot)$](./readme_assets/eq/PP.jpg) projects each of the ![$K$](./readme_assets/eq/kk.jpg) world points ![$\mathcal{Q}(\cdot)$](./readme_assets/eq/QQ.jpg) onto camera's image plane, and ![$\mathcal{N}$](./readme_assets/eq/N.jpg) is for averaging.


<h2 id="vae">VAE outlier corner detection</h2>

---

We treat incorrectly or inaccurately detected corners as
outliers and identify them using VAE.

<img src="./readme_assets/corner_example.jpg" style="max-width:500px; width=500px">

<h3 id="vae-objective">Objective of VAE</h3>

VAE deals with an unknown underlying probabilistic distribution ![$p^*(\bb{x})$](./readme_assets/eq/vae/p_star.jpg) defined over the data points ![$\bb{x}$](./readme_assets/eq/vae/x.jpg) in some potentially high-dimensional space ![$\mathcal{X}$](./readme_assets/eq/vae/xx.jpg)<sup>[[2]](#kingma2013auto)</sup>. The aim of VAE is to approximate the underlying distribution with a chosen model ![$p_{\bb{\theta}}(\bb{x}) \approx p^*(\bb{x})$](./readme_assets/eq/vae/p_theta_approx.jpg), parameterized by ![$\bb{\theta}$](./readme_assets/eq/vae/theta.jpg) and marginalized over the latent variables ![$\bb{z}$](./readme_assets/eq/vae/z.jpg):

![$\bb{x}\sim p_{\bb{\theta}}(\bb{x}) = \int p_{\bb{\theta}}(\bb{x}, \bb{z}) d\bb{z} = \int p_{\bb{\theta}}(\bb{x}|\bb{z}) p_{\bb{\theta}}(\bb{z})d\bb{z}$](./readme_assets/eq/vae/x_approx.jpg).

The objective is to estimate the parameters ![$\bb{\theta}$](./readme_assets/eq/vae/theta.jpg) via Maximum Likelihood Estimation (MLE) on the likelihood of the data set ![$p_{\bb{\theta}}(\bb{X})$](./readme_assets/eq/vae/p_theta_xx.jpg) consisting of i.i.d. samples ![$\bb{X} = \{ \bb{x}_i \}_{i=1}^N$](./readme_assets/eq/vae/x_set.jpg). Using variational inference for approximating the intractable true posterior ![$p_{\theta}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/p_theta_zx.jpg) involved in ![$p_{\theta}(\bb{x}_i)$](./readme_assets/eq/vae/p_theta_xi.jpg) with some tractable distribution ![$q_{\phi}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/q_theta_zx.jpg), then it is sufficient for MLE to maximize the evidence lower bound ![$\mathcal{L}_{\bb{\theta}, \bb{\phi}}(\bb{x}_i)$](./readme_assets/eq/vae/L.jpg):

![lower_bound](./readme_assets/eq/vae/lower_bound.jpg),

where ![$D_{KL}(\cdot)$](./readme_assets/eq/vae/dkl.jpg) is Kullback-Leibler (KL) divergence<sup>[[3]](#murphy2012machine)</sup>.

To make the maximization of the evidence lower bound tractable, the prior ![$p_{\bb{\theta}}(\bb{z})$](./readme_assets/eq/vae/p_theta_z.jpg) is set as isotropic unit Gaussian and the posterior ![$q_{\bb{\phi}}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/q_phi_zxi.jpg) as Gaussians ![$\mathcal{N}(\bm{\mu}_q, \sigma_q^2\bm{I})$](./readme_assets/eq/vae/N_dist.jpg). Also, this parameterization allows the use of the *reparameterization trick* to make the random sampling of ![$\bb{z}$$\sim$$q_{\bb{\phi}}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/z_approx.jpg) in ![$\mathcal{L}_{\bb{\theta}, \bb{\phi}}(\bb{x}_i)$](./readme_assets/eq/vae/L.jpg) differentiable w.r.t. ![$\bb{\phi}$](./readme_assets/eq/vae/phi.jpg) by reparameterizing it as ![$\bb{z} = \bm{\mu}_q + \sigma_q \bm{\epsilon}$](./readme_assets/eq/vae/z_equal.jpg), with ![$\bm{\epsilon}$$\sim$$\mathcal{N}(\bb{0}, \bb{I})$](./readme_assets/eq/vae/epsilon_approx.jpg). Moreover, by choosing ![$p_{\bb{\theta}}(\bb{x}_i|\bb{z})$$\sim$$\mathcal{N}(\bm{\mu}_p, \sigma_p^2 \bm{I})$](./readme_assets/eq/vae/p_theta_xz_approx.jpg) where ![$\sigma_p$](./readme_assets/eq/vae/sigma_p.jpg) is pre-determined and after collecting constant terms into ![$\omega$](./readme_assets/eq/vae/omega.jpg), maximizing ![the right-hand side of Equation \ref{eq:lower-bound}](./readme_assets/eq/vae/L.jpg) becomes equivalent to minimizing:

![vae_obj](./readme_assets/eq/vae/vae_obj.jpg)

For VAEs, ![$q_\phi(\bb{z}|\bb{x})$](./readme_assets/eq/vae/q_phi_zx.jpg) refers to a probabilistic *encoder* and ![$p_\theta(\bb{x}|\bb{z})$](./readme_assets/eq/vae/p_theta_xz.jpg) to a probabilistic *decoder*.


<h3 id="vae-architecture">VAE architecture</h3>

<img src="./readme_assets/eq/vae/vae_architecture.jpg" style="max-width:600px; width=600px">

For input images, we first remove shading information through binarization using Otsu's method following Gaussian blurring. The last layer in the decoder is a sigmoid activation which returns pixel values in range [0,1].

<h2 id="experimental">Experimental results</h2>

---

<h3 id="vae-effects">Effects of corner outliers on the calibration accuracy</h3>

<figure>
    <img src="./readme_assets/studio_setup.jpg" width="450px" id="studio-setup">
    <figcaption>Figure 1: (a) 16 camera studio setup and (b) its virtual counterpart with a randomly moving checkerboard visualized.</figcaption>
</figure>

We generate four sets of synthetic image points of a virtual planar checkerboard ([Figure 1](#studio-setup)). We include different percentages of outlier corners (0%, 0.01%, 0.1%, and 1%) in each image set where each outlier corner is randomly offset from its ground-truth location by 10-15 pixels. Then, we estimate the parameters of 16 cameras using each of the four image sets and compare the root mean square error (RMSE) between the ground-truth.

<figure>
    <img src="./readme_assets/cam_param_compare_outlier.jpg" width="600px" id="rmse">,
    <figcaption>Figure 2: RMSE comparisons of estimated camera parameters, grouped by percentages of outlier corners.</figcaption>
</figure>

The result is obtained as in [Figure 2](#rmse), where the geodesic distance on unit sphere is used for computing the error in camera orientations ![$\bb{R}'$](./readme_assets/eq/exp/R_dash.jpg), and ![$\bm{t}'$](./readme_assets/eq/exp/t_dash.jpg) denotes camera position. Since each calibration is obtained in an arbitrary coordinates system, we align them with the reference coordinates using Procrustes analysis before computing RMSE.

<h3 id="vae-performance">Performance evaluations</h3>

<h4 id="performance-synth">Synthetic experiment</h4>

We generate 10k synthetic corner images with 1% outliers by cropping 15x15 pixels from the template corner after applying random affine transformations:

<img src="./readme_assets/corner_template.jpg" width="120px">

The outlier corners are offset from the center of crop by 2-4 pixels ([Figure 3b](#synth-crops)).

We set the latent space of VAE and AE to 2 dimensional, and VAE's KL divergence weight to ![$\omega$](./readme_assets/eq/vae/omega.jpg)=1. Then, VAE and AE are trained for 1k epochs using Adam optimizer with learning rate 1e-3. For PCA and kPCA (with a Gaussian kernel), we use 2 principal axes for reconstructions.

We use the same binarized inputs for both training and testing ([Figure 3](#synth-crops)) and compare the results using the area under the receiver operator characteristic curve (AUROC) and the precision recall curve (AUPRC). Note, when the data for binary classification is heavily imbalanced, AUPRC can be less misleading than AUROC<sup>[[4]](#davis2006relationship)</sup>.

<figure>
    <img src="./readme_assets/vae_synth_crops.jpg" width="600px" id="synth-crops">
    <figcaption>Figure 3: Reconstructions and normalized losses of synthetic corners: (a) inliers and (b) outliers.</figcaption>
</figure>

From [Figure 4](#auroc-auprc), we observe AUPRC of VAE is the largest, outperforming the rest. For VAE, the precision ratio remains very close to 1 even as the recall ratio approaches 1, implying VAE can remove outliers without also removing the inliers.

<figure>
    <img src="./readme_assets/auroc_auprc.jpg" width="600px" id="auroc-auprc">
    <figcaption>Figure 4: Area under the receiver operator characteristic curve (AUROC) and the precision recall curve (AUPRC).</figcaption>
</figure>

The performance differences are more pronounced in the normalized reconstruction losses plot of every data points in [Figure 5](#recon-loss). In contrast to AE, PCA, and kPCA, a clear classification margin is achieved by VAE which implies the outliers can be identified with high confidence.

<figure>
    <img src="./readme_assets/recon_loss.jpg" width="600px" id="recon-loss">
    <figcaption>Figure 5: Normalized reconstruction losses of every corner crops by VAE, AE, PCA, and kPCA.</figcaption>
</figure>

<h4 id="performance-real">Real experiment</h4>

We capture images (~172 frames per camera) of a freely moving checkerboard and crop 15x15 pixels centered at detected corners. Similarly, we train VAE with heuristically determined KL divergence weight ![$\omega$](./readme_assets/eq/vae/omega.jpg)=0.01 for 200 epochs.

<figure>
    <img src="./readme_assets/recons_real.jpg" width="600px" id="recons-real">
    <figcaption>Figure 6: Original grayscale crops (above) and VAE reconstructions (below) sorted by normalized losses.</figcaption>
</figure>

<figure>
    <img src="./readme_assets/recon_loss_real.jpg" width="600px" id="recon-loss-real">
    <figcaption>Figure 7: Normalized VAE reconstruction losses of every corner crops displaying distinct classification margin.</figcaption>
</figure>

The result in [Figure 6](#recons-real) shows that the outlier corners are reconstructed with large loss, and via visual inspection 37 outliers can be identified (~ 0.015% outliers). Similar to [Figure 5](#recon-loss), a clear classification margin can be observed in the reconstruction loss plot in [Figure 7](#recon-loss-real). Utilizing these, we can determine the outlier corners with high confidence from tens to thousands of images.

<h2 id="references">References</h2>

---
<a id="szeliski2010computer">[1]</a> Szeliski R. Computer vision: algorithms and applications. Springer Science & Business Media; 2010 Sep 30.

<a id="kingma2013auto">[2]</a> Kingma DP, Welling M. Auto-encoding variational bayes. arXiv preprint arXiv:1312.6114. 2013 Dec 20.

<a id="murphy2012machine">[3]</a> Murphy KP. Machine learning: a probabilistic perspective. MIT press; 2012 Sep 7.

<a id="davis2006relationship">[4]</a> Davis J, Goadrich M. The relationship between Precision-Recall and ROC curves. InProceedings of the 23rd international conference on Machine learning 2006 Jun 25 (pp. 233-240).

