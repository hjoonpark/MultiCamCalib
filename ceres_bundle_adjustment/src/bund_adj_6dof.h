#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "parser.h"
#include "checkerboard.h"
#include <Eigen/Dense>

class LossFunction {
public:
	virtual void Evaluate(double s, double out[3]) const = 0;
};

class BundAdj6Dof {
public:
    std::vector<Camera> cameras;
    std::vector<Frame> frames;
    std::vector<Checkerboard> checkerboards;
    BundAdj6Dof::Config config;

    void compute_center_cam_dR_dt(double (&rvec0)[3], double (&tvec0)[3], double (&rvec1)[3], double (&tvec1)[3], double (&d_rvec)[3], double (&d_tvec)[3]) {
        Eigen::Vector3d r0(rvec0[0], rvec0[1], rvec0[2]);
        Eigen::Vector3d r1(rvec1[0], rvec1[1], rvec1[2]);

        Eigen::Matrix3d R0;
        R0 = Eigen::AngleAxisd(r0.norm(), r0 / r0.norm());
        Eigen::Matrix3d R1;
        R1 = Eigen::AngleAxisd(r1.norm(), r1 / r1.norm());

        Eigen::Vector3d t0(tvec0[0], tvec0[1], tvec0[2]);
        Eigen::Vector3d t1(tvec1[0], tvec1[1], tvec1[2]);

        // extrinsics -> SE(3)
        /*
        t0 = -R0.transpose() * t0;
        t1 = -R1.transpose() * t1;
        R0 = R0.transpose();
        R1 = R1.transpose();

        Eigen::Matrix3d dR = R0.transpose() * R1;
        Eigen::Vector3d dt = R0.transpose() * (t1 - t0);
        */
        
        Eigen::Matrix3d dR;
        dR << 1, 0, 0, 0, 0, -1, 0, 1, 0;
        Eigen::Vector3d dt(0, 0, 0);

        Eigen::AngleAxisd d_r_aa(dR);
        Eigen::Vector3d dr = d_r_aa.axis() * d_r_aa.angle();
        
        d_rvec[0] = dr(0);
        d_rvec[1] = dr(1);
        d_rvec[2] = dr(2);

        d_tvec[0] = dt(0);
        d_tvec[1] = dt(1);
        d_tvec[2] = dt(2);
    }

    void retarget_cameras(std::vector<Camera> &cameras, double (&drvec)[3], double (&dtvec)[3]) {
        Eigen::Vector3d dr(drvec[0], drvec[1], drvec[2]);
        Eigen::Vector3d dt(dtvec[0], dtvec[1], dtvec[2]);
        Eigen::Matrix3d dR;
        dR = Eigen::AngleAxisd(dr.norm(), dr / dr.norm());
        dR = Eigen::Matrix3d::Identity();

        for(size_t cam_idx = 0; cam_idx < cameras.size(); cam_idx++) {
            Camera *cam = &cameras[cam_idx];

            Eigen::Vector3d r0(cam->rvec()[0], cam->rvec()[1], cam->rvec()[2]);
            Eigen::Matrix3d R0;
            R0 = Eigen::AngleAxisd(r0.norm(), r0 / r0.norm());
            Eigen::Vector3d t0(cam->tvec()[0], cam->tvec()[1], cam->tvec()[2]);

            // extrinsics -> SE3
            t0 = -R0.transpose() * t0;
            R0 = R0.transpose();

            Eigen::Matrix3d R1 = dR*R0;
            Eigen::Vector3d tvec_after = dR * t0 + dt;

            R1 = R0;
            tvec_after = t0;

            // SE3 -> extrinsics
            tvec_after = -R1.transpose() * tvec_after;
            R1 = R1.transpose();

            Eigen::AngleAxisd r1(R1);
            Eigen::Vector3d rvec_after = r1.axis() * r1.angle();

            std::cout << "Cam" << cam_idx << std::endl;
            std::cout << R1 << std::endl << std::endl;
            std::cout << r1.axis() << "\t" << r1.angle() << std::endl;
            std::cout << rvec_after << std::endl;
            int k;
            std::cin >> k;

            cam->rvec()[0] = rvec_after(0);
            cam->rvec()[1] = rvec_after(1);
            cam->rvec()[2] = rvec_after(2);

            cam->tvec()[0] = tvec_after(0);
            cam->tvec()[1] = tvec_after(1);
            cam->tvec()[2] = tvec_after(2);
        }
    }

    void init(const Config &config_) {
        config = config_;
        Checkerboard::initGlobalProperties(config.chb_n_rows, config.chb_n_cols, config.chb_sqr_size);

        std::stringstream cam_path;
        cam_path << config.dir_cam_params << OS_SEP << "cam_params_initial.json";
        std::cout << "Load camera parameters: " << cam_path.str() << std::endl;
        Parser::loadInitialCamParams(cam_path.str().c_str(), config.n_cams, cameras);
        std::cout << "  - " << cameras.size() << " cameras loaded" << std::endl;

        std::stringstream detection_result_path;
        detection_result_path << config.dir_corners << OS_SEP << "detection_result.json";
        std::stringstream outliers_path;
        outliers_path << config.dir_outliers << OS_SEP << "outliers.json";
        Parser::loadDetectionResult(outliers_path.str().c_str(), detection_result_path.str().c_str(), frames, config.n_cams);
        std::cout << "  - " << frames.size() << " vaild frames loaded" << std::endl;
        
        std::stringstream world_points_path;
        world_points_path << config.dir_world_points << OS_SEP << "world_points_initial.json";
        Parser::loadInitialCheckerboardPoses(world_points_path.str().c_str(), frames, checkerboards);
        std::cout << "  - " << checkerboards.size() << " chb poses loaded" << std::endl;
    }

    void run(const Config &config) {
        std::cout << ">> Run" << std::endl;
        // loss function
        ceres::LossFunction *loss = NULL; 

        // save initial pose of center camera for retargeting after bundle adjustment.
        Camera *center_cam = &cameras[config.center_cam_idx];
        double rvec_center_initial[3] = {center_cam->rvec()[0], center_cam->rvec()[1], center_cam->rvec()[2]};
        double tvec_center_initial[3] = {center_cam->tvec()[0], center_cam->tvec()[1], center_cam->tvec()[2]};

        int n_cams = cameras.size();
        int n_residual_blocks = 0;
        ceres::Problem ceres_prob;
        for (int frame_idx = 0; frame_idx < frames.size(); frame_idx++) {
            Frame *frame = &frames[frame_idx];
            Checkerboard *chb = &checkerboards[frame_idx];
            for(int cam_idx = 0; cam_idx < n_cams; cam_idx++ ){
                if(frame->detected[cam_idx]) {
                    // image point path
                    std::stringstream image_points_path;
                    image_points_path << config.dir_corners << OS_SEP << "cam_" << std::to_string(cam_idx) << OS_SEP << std::to_string(cam_idx) << "_" << frame->img_name.c_str() << ".txt";

                    // ceres::CostFunction *cost_func = BundAdj6Dof::ReprojectionError::Create(imgpts_path.str().c_str(), chb, frame->n_detected);
                    BundAdj6Dof::ReprojectionError::ReprojectionErrorCostFunc *cost_func = BundAdj6Dof::ReprojectionError::Create(image_points_path.str().c_str(), chb, frame->n_detected);

                    std::vector<double*> parameter_blocks;
                    parameter_blocks.push_back(cameras[cam_idx].params);
                    parameter_blocks.push_back(chb->rvec);
                    parameter_blocks.push_back(chb->tvec);
                    ceres_prob.AddResidualBlock(cost_func, loss, parameter_blocks);
                    n_residual_blocks += 1;
                }
            }
        }
        
        // regularization
        double lens_coeffs_weights[5] = {0.1};
        for (int cam_idx = 0; cam_idx < n_cams; cam_idx++) {
            ceres::CostFunction *reg_func = BundAdj6Dof::LensDistortionRegularization::Create(lens_coeffs_weights);
            ceres_prob.AddResidualBlock(reg_func, NULL, cameras[cam_idx].params);
            n_residual_blocks += 1;
        }

        std::cout << ">> " << n_residual_blocks << " residual blocks." << std::endl;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = 1;
        options.max_num_iterations = config.max_iter;
        options.num_threads = config.num_thread;
        options.function_tolerance = config.function_tolerance;
        options.parameter_tolerance = config.parameter_tolerance;
        options.gradient_tolerance = config.gradient_tolerance;
        options.inner_iteration_tolerance = config.inner_iteration_tolerance;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &ceres_prob, &summary);
        std::cout << summary.FullReport() << std::endl;
        std::cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << std::endl;

        // recenter all cameras s.t. center camera is set to its initial pose
        double d_rvec[3], d_tvec[3];
        double rvec_center_final[3] = {center_cam->rvec()[0], center_cam->rvec()[1], center_cam->rvec()[2]};
        double tvec_center_final[3] = {center_cam->tvec()[0], center_cam->tvec()[1], center_cam->tvec()[2]};
        compute_center_cam_dR_dt(rvec_center_initial, tvec_center_initial, rvec_center_final, tvec_center_final, d_rvec, d_tvec);
        retarget_cameras(cameras, d_rvec, d_tvec);

        // save outputs
        std::stringstream cam_out_path;
        cam_out_path << config.dir_cam_params << OS_SEP << "cam_params_final.json";
        Parser::saveFinalCameraParameters(cam_out_path.str().c_str(), cameras);
        std::cout << ">> Camera parameters saved: " << cam_out_path.str() << std::endl;

        // save final world points
        std::stringstream world_points_out_path;
        world_points_out_path << config.dir_world_points << OS_SEP << "world_points_final.json";
        Parser::saveFinalWorldPoints(world_points_out_path.str().c_str(), checkerboards, frames);
        std::cout << ">> World points saved: " << world_points_out_path.str() << std::endl;

        // save bundle adjustment result
        std::stringstream result_out_path;
        result_out_path << config.dir_ceres_output << OS_SEP << "bundle_adjustment_result.json";
        Parser::saveBundleAdjustmentResult(result_out_path.str().c_str(), summary.initial_cost, summary.final_cost, summary.iterations.back().iteration, cam_out_path.str().c_str(), world_points_out_path.str().c_str());
        std::cout << ">> Bundle adjustment result saved: " << result_out_path.str() << std::endl;
    }

    struct LensDistortionRegularization {
        double *weights;
        LensDistortionRegularization(double (&weights_)[5]) {
            weights = weights_;
        }
        static ceres::CostFunction* Create(double (&weights_)[5]) {
            constexpr int kResidualDim = 5;
            constexpr int kNumCamParams = NUM_CAM_PARAMS;
            LensDistortionRegularization* re = new BundAdj6Dof::LensDistortionRegularization(weights_);
            return (new ceres::AutoDiffCostFunction<LensDistortionRegularization, kResidualDim, kNumCamParams>(re));
        }
        ~LensDistortionRegularization() {
            delete[] weights;
        }
        
		template<typename T>
		bool operator()(const T* const cam_params, T* residuals) const {
            residuals[0] = weights[0]*cam_params[K1];
            residuals[1] = weights[1]*cam_params[K2];
            residuals[2] = weights[2]*cam_params[P1];
            residuals[3] = weights[3]*cam_params[P2];
            residuals[4] = weights[4]*cam_params[K3];
            return true;
        }
    };

    #define CERES_STRIDE 4
    struct ReprojectionError {
        Checkerboard *chb;
        double *img_pts;
        int n_cams_used;

        ReprojectionError(const char* imgpts_path_, Checkerboard *chb_, const int n_cams_used_){
            n_cams_used = n_cams_used_;
            chb = chb_;
            img_pts = new double[2*chb->n_pts];
            Parser::loadImagePoints(imgpts_path_, img_pts);
        }

        typedef ceres::DynamicAutoDiffCostFunction<ReprojectionError, CERES_STRIDE> ReprojectionErrorCostFunc;
        static ReprojectionErrorCostFunc* Create(const char* imgpts_path_, Checkerboard *chb_, const int n_cams_used_) {
            ReprojectionError* re = new BundAdj6Dof::ReprojectionError(imgpts_path_, chb_, n_cams_used_);
            ReprojectionErrorCostFunc* cost_func = new ReprojectionErrorCostFunc(re);
            
            constexpr int kNumCamParams = NUM_CAM_PARAMS;
            constexpr int kChbRvecDim = 3;
            constexpr int kChbTvecDim = 3;
            cost_func->AddParameterBlock(kNumCamParams);
            cost_func->AddParameterBlock(kChbRvecDim);
            cost_func->AddParameterBlock(kChbTvecDim);
            cost_func->SetNumResiduals(2*chb_->n_pts);
            return cost_func;
        }

        /*
        static ceres::CostFunction* Create(const char* imgpts_path_, Checkerboard *chb_, const int n_cams_used_) {
            constexpr int kChbRvecDim = 3;
            constexpr int kChbTvecDim = 3;
            constexpr int kNumResiduals = 2;
            ReprojectionError *re = new BundAdj6Dof::ReprojectionError(imgpts_path_, chb_, n_cams_used_);
            return (new ceres::DynamicAutoDiffCostFunction<ReprojectionError, 176, NUM_CAM_PARAMS, kChbRvecDim, kChbTvecDim>(re));
        }
        */

        ~ReprojectionError() {
            delete[] img_pts;
        }

		template<typename T>
		bool operator()(T const* const* params, T* residuals) const {
            // camera parameters
			T extrinsics_rvec[3] = { params[0][RVEC], params[0][RVEC+1], params[0][RVEC+2] };
			T extrinsics_tvec[3] = { params[0][TVEC], params[0][TVEC+1], params[0][TVEC+2] };
			T f[2] = { params[0][FX], params[0][FY] };
			T c[2] = { params[0][CX], params[0][CY] };
            T k[3] = { params[0][K1], params[0][K2], params[0][K3]};
            T p[2] = { params[0][P1], params[0][P2]};

            // checkerboard
            T chb_ang_axis[3] = {params[1][0], params[1][1], params[1][2]};
            T chb_trans[3] = {params[2][0], params[2][1], params[2][2]};

            for (size_t i = 0; i < chb->n_pts; i++) {
                // update checkerboard points (6 dof)
                const T chb_point[3] = {T(chb->chb_pts[i*3]), T(chb->chb_pts[i*3+1]), T(chb->chb_pts[i*3+2])};

                T world_point[3];
                ceres::AngleAxisRotatePoint(chb_ang_axis, chb_point, world_point);
                world_point[0] += chb_trans[0];
                world_point[1] += chb_trans[1];
                world_point[2] += chb_trans[2];

                // model to camera frame
                T cam_point[3];
                ceres::AngleAxisRotatePoint(extrinsics_rvec, world_point, cam_point);
                cam_point[0] += extrinsics_tvec[0];
                cam_point[1] += extrinsics_tvec[1];
                cam_point[2] += extrinsics_tvec[2];

                // center of distortion
                T xp = cam_point[0] / cam_point[2];
                T yp = cam_point[1] / cam_point[2];

                // lens distortions
                T r2 = xp*xp + yp*yp;
                T rad_dist = T(1) + k[0]*r2 + k[1]*r2*r2 + k[2]*r2*r2*r2;
                T tan_dist[2] ={ p[1]*(r2+2.0*xp*xp) + 2.0*p[0]*xp*yp, p[0]*(r2+2.0*yp*yp) + 2.0*p[1]*xp*yp };

                T u = xp * rad_dist + tan_dist[0];
                T v = yp * rad_dist + tan_dist[1];
                //T u = xp;
                //T v = yp;

                // projection to image
                T x_pred = f[0]*u + c[0];
                T y_pred = f[1]*v + c[1];

                // reprojection error
                T dx = x_pred - img_pts[i*2];
                T dy = y_pred - img_pts[i*2 + 1];

                // output
                residuals[2*i] = dx / T(n_cams_used);
                residuals[2*i + 1] = dy / T(n_cams_used);
            }
            return true;
        }
    };
};
