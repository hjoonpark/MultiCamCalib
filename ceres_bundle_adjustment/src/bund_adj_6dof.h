#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <map>

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
    std::map<std::string, int> imgname_to_frameidx;
    BundAdj6Dof::Config config;

    void compute_center_dR_dt(double (&rvec_chb)[3], double (&tvec_chb)[3], Eigen::Matrix3d &dR, Eigen::Vector3d &dt) {
        Eigen::Vector3d r_chb(rvec_chb[0], rvec_chb[1], rvec_chb[2]);

        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(r_chb.norm(), r_chb / r_chb.norm());
        Eigen::Vector3d t(tvec_chb[0], tvec_chb[1], tvec_chb[2]);

        // dR, dt
        dR = R.transpose();
        dt = -R.transpose() * t;
    }

    void retarget_cameras(std::vector<Camera> &cameras, const Eigen::Matrix3d &dR, const Eigen::Vector3d &dt) {
        for(size_t cam_idx = 0; cam_idx < cameras.size(); cam_idx++) {
            Camera *cam = &cameras[cam_idx];

            // extrinsics
            Eigen::Vector3d r(cam->rvec()[0], cam->rvec()[1], cam->rvec()[2]);
            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(r.norm(), r / r.norm());
            Eigen::Vector3d t(cam->tvec()[0], cam->tvec()[1], cam->tvec()[2]);

            // new extrinsics
            Eigen::Matrix3d R_new = R * dR.transpose();
            Eigen::Vector3d tvec_new = -R * dR.transpose() * dt + t;
            
            Eigen::AngleAxisd r_new_aa(R_new);
            Eigen::Vector3d rvec_new = r_new_aa.axis() * r_new_aa.angle();

            cam->rvec()[0] = rvec_new(0);
            cam->rvec()[1] = rvec_new(1);
            cam->rvec()[2] = rvec_new(2);

            cam->tvec()[0] = tvec_new(0);
            cam->tvec()[1] = tvec_new(1);
            cam->tvec()[2] = tvec_new(2);
        }
    }

    void retarget_checkerboards(std::vector<Checkerboard>& chbs, const Eigen::Matrix3d& dR, const Eigen::Vector3d& dt) {
        for (size_t frame_idx = 0; frame_idx < chbs.size(); frame_idx++) {
            Checkerboard* chb = &chbs[frame_idx];

            // se3
            Eigen::Vector3d r(chb->rvec[0], chb->rvec[1], chb->rvec[2]);
            Eigen::Matrix3d R;
            R = Eigen::AngleAxisd(r.norm(), r / r.norm());
            Eigen::Vector3d t(chb->tvec[0], chb->tvec[1], chb->tvec[2]);

            // new se3
            Eigen::Matrix3d R_new = dR * R;
            Eigen::Vector3d tvec_new = dR * t + dt;

            // back to rvec, tvec
            Eigen::AngleAxisd r_aa(R_new);
            Eigen::Vector3d rvec_new = r_aa.axis() * r_aa.angle();
        
            chb->rvec[0] = rvec_new[0];
            chb->rvec[1] = rvec_new[1];
            chb->rvec[2] = rvec_new[2];

            chb->tvec[0] = tvec_new[0];
            chb->tvec[1] = tvec_new[1];
            chb->tvec[2] = tvec_new[2];
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
        Parser::loadInitialCheckerboardPoses(world_points_path.str().c_str(), frames, checkerboards, imgname_to_frameidx);
        std::cout << "  - " << checkerboards.size() << " chb poses loaded" << std::endl;
    }

    void run(const Config &config) {
        std::cout << ">> Run" << std::endl;
        // loss function
        ceres::LossFunction *loss = NULL; 


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
        double lens_coeffs_weights[5] = {0.01};
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

        // recenter all cameras & checkerboards s.t. center checkerboard is set to its initial pose
        int center_frame_idx = imgname_to_frameidx[config.center_img_name];
        Checkerboard* center_chb = &checkerboards[center_frame_idx];
        double rvec_center[3] = { center_chb->rvec[0], center_chb->rvec[1], center_chb->rvec[2] };
        double tvec_center[3] = { center_chb->tvec[0], center_chb->tvec[1], center_chb->tvec[2] };
        Eigen::Matrix3d dR;
        Eigen::Vector3d dt;
        compute_center_dR_dt(rvec_center, tvec_center, dR, dt);
        retarget_cameras(cameras, dR, dt);
        retarget_checkerboards(checkerboards, dR, dt);

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
