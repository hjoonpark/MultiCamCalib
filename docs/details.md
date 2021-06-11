<h1>Academical details</h1>

[Camera model](#camera_model)

[VAE outlier corner detection](#emphasis)  



<h2 id="camera_model">Camera model</h2>

---

We model the cameras as the usual pinhole camera. A 3D world point is denoted as ![$p=[x, y, z]$](./readme_assets/eq/world_point.jpg) and its projected image point as ![$q=[u, v]$](./readme_assets/eq/image_point.jpg), related by:
![pinhole-model](./readme_assets/eq/camera_model.jpg),

where ![$(R, t)$](./readme_assets/eq/rt.jpg) are extrinsic parameters with rotation (parameterized as the angle-axis vector) and translation, and ![s](./readme_assets/eq/s.jpg) is an arbitrary scale factor. ![K](./readme_assets/eq/k.jpg) is the camera intrinsic matrix with focal lengths ![$(fx, fy)$](./readme_assets/eq/fxfy.jpg) and principal points ![$(cx, cy)$](./readme_assets/eq/cxcy.jpg) in pixel unit. ![$\gamma$](./readme_assets/eq/gamma.jpg) describes the skew of the two image axes which can be assumed zero for most modern cameras<sup>[[1]](#szeliski2010computer)</sup>. 

The lens distortions are modeled using the coefficients ![$(k1, k2, k3)$](./readme_assets/eq/k1k2k3.jpg) for radial and ![$(p1, p2)$](./readme_assets/eq/p1p2.jpg) for tangential distortions following the widely used lens model<sup>[[1]](#szeliski2010computer)</sup> relating a distorted image point ![$q_d$](./readme_assets/eq/q_d.jpg) and undistorted image point ![$q$](./readme_assets/eq/q.jpg) by:
![lens-model](./readme_assets/eq/lens_distortion.jpg)

As a result, each camera is modeled with 15 parameters
(6 extrinsics, 4 intrinsics, and 5 lens distortion parameters).

<h2 id="bundle_adjustment">Bundle adjustment with planar checkboard</h2>

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
<img src="./readme_assets/corner_example.jpg" style="max-width:500px; width=100%">

<h3>Objective of VAE</h3>

VAE deals with an unknown underlying probabilistic distribution ![$p^*(\bb{x})$](./readme_assets/eq/vae/p_star.jpg) defined over the data points ![$\bb{x}$](./readme_assets/eq/vae/x.jpg) in some potentially high-dimensional space ![$\mathcal{X}$](./readme_assets/eq/vae/xx.jpg)<sup>[[2]](#kingma2013auto)</sup>. The aim of VAE is to approximate the underlying distribution with a chosen model ![$p_{\bb{\theta}}(\bb{x}) \approx p^*(\bb{x})$](./readme_assets/eq/vae/p_theta_approx.jpg), parameterized by ![$\bb{\theta}$](./readme_assets/eq/vae/theta.jpg) and marginalized over the latent variables ![$\bb{z}$](./readme_assets/eq/vae/z.jpg):

![$\bb{x}\sim p_{\bb{\theta}}(\bb{x}) = \int p_{\bb{\theta}}(\bb{x}, \bb{z}) d\bb{z} = \int p_{\bb{\theta}}(\bb{x}|\bb{z}) p_{\bb{\theta}}(\bb{z})d\bb{z}$](./readme_assets/eq/vae/x_approx.jpg).

The objective is to estimate the parameters ![$\bb{\theta}$](./readme_assets/eq/vae/theta.jpg) via Maximum Likelihood Estimation (MLE) on the likelihood of the data set ![$p_{\bb{\theta}}(\bb{X})$](./readme_assets/eq/vae/p_theta_xx.jpg) consisting of i.i.d. samples ![$\bb{X} = \{ \bb{x}_i \}_{i=1}^N$](./readme_assets/eq/vae/x_set.jpg). Using variational inference for approximating the intractable true posterior ![$p_{\theta}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/p_theta_zx.jpg) involved in ![$p_{\theta}(\bb{x}_i)$](./readme_assets/eq/vae/p_theta_xi.jpg) with some tractable distribution ![$q_{\phi}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/q_theta_zx.jpg), then it is sufficient for MLE to maximize the evidence lower bound ![$\mathcal{L}_{\bb{\theta}, \bb{\phi}}(\bb{x}_i)$](./readme_assets/eq/vae/L.jpg):

![lower_bound](./readme_assets/eq/vae/lower_bound.jpg),

where ![$D_{KL}(\cdot)$](./readme_assets/eq/vae/dkl.jpg) is Kullback-Leibler (KL) divergence<sup>[[3]](#murphy2012machine)</sup>.

To make the maximization of the evidence lower bound tractable, the prior ![$p_{\bb{\theta}}(\bb{z})$](./readme_assets/eq/vae/p_theta_z.jpg) is set as isotropic unit Gaussian and the posterior ![$q_{\bb{\phi}}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/q_phi_zxi.jpg) as Gaussians ![$\mathcal{N}(\bm{\mu}_q, \sigma_q^2\bm{I})$](./readme_assets/eq/vae/N_dist.jpg). Also, this parameterization allows the use of the *reparameterization trick* to make the random sampling of ![$\bb{z}$$\sim$$q_{\bb{\phi}}(\bb{z}|\bb{x}_i)$](./readme_assets/eq/vae/z_approx.jpg) in ![$\mathcal{L}_{\bb{\theta}, \bb{\phi}}(\bb{x}_i)$](./readme_assets/eq/vae/L.jpg) differentiable w.r.t. ![$\bb{\phi}$](./readme_assets/eq/vae/phi.jpg) by reparameterizing it as ![$\bb{z} = \bm{\mu}_q + \sigma_q \bm{\epsilon}$](./readme_assets/eq/vae/z_equal.jpg), with ![$\bm{\epsilon}$$\sim$$\mathcal{N}(\bb{0}, \bb{I})$](./readme_assets/eq/vae/epsilon_approx.jpg). Moreover, by choosing ![$p_{\bb{\theta}}(\bb{x}_i|\bb{z})$$\sim$$\mathcal{N}(\bm{\mu}_p, \sigma_p^2 \bm{I})$](./readme_assets/eq/vae/p_theta_xz_approx.jpg) where ![$\sigma_p$](./readme_assets/eq/vae/sigma_p.jpg) is pre-determined and after collecting constant terms into ![$\omega$](./readme_assets/eq/vae/omega.jpg), maximizing ![the right-hand side of Equation \ref{eq:lower-bound}](./readme_assets/eq/vae/L.jpg) becomes equivalent to minimizing:

![vae_obj](./readme_assets/eq/vae/vae_obj.jpg)

For VAEs, ![$q_\phi(\bb{z}|\bb{x})$](./readme_assets/eq/vae/q_phi_zx.jpg) refers to a probabilistic *encoder* and ![$p_\theta(\bb{x}|\bb{z})$](./readme_assets/eq/vae/p_theta_xz.jpg) to a probabilistic *decoder*.


<h3>VAE architecture</h3>
<img src="./readme_assets/eq/vae/vae_architecture.jpg" style="max-width:600px; width=100%">

For input images, we first remove shading information through binarization using Otsu's method following Gaussian blurring. The last layer in the decoder is a sigmoid activation which returns pixel values in range [0,1].

<h2 id="references">References</h2>

---
<a id="szeliski2010computer">[1]</a> Szeliski R. Computer vision: algorithms and applications. Springer Science & Business Media; 2010 Sep 30.

<a id="kingma2013auto">[2]</a> Kingma DP, Welling M. Auto-encoding variational bayes. arXiv preprint arXiv:1312.6114. 2013 Dec 20.

<a id="murphy2012machine">[3]</a> Murphy KP. Machine learning: a probabilistic perspective. MIT press; 2012 Sep 7.



