#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "parser.h"
#include "checkerboard.h"

class LossFunction {
public:
	virtual void Evaluate(double s, double out[3]) const = 0;
};

class BundAdj6Dof {
public:
    Checkerboard *checkerboard;
    std::vector<Camera> cameras;
    std::vector<Frame> frames;
    std::vector<std::array<double, 6>> chb_poses;

    void init(const char* output_dir, const int n_cams, const int n_rows, const int n_cols, const float sqr_size) {
        checkerboard = new Checkerboard(n_rows, n_cols, sqr_size);

        std::stringstream cam_path;
        cam_path << output_dir << OS_SEP << "cam_params" << OS_SEP << "cam_params_initial.json";
        std::cout << "Read: " << cam_path.str() << std::endl;
        Parser::loadInitialCamParams(cam_path.str().c_str(), n_cams, cameras);
        std::cout << "  - " << cameras.size() << " cameras loaded" << std::endl;

        std::stringstream detect_res_path;
        detect_res_path << output_dir << OS_SEP << "detection_result.json";
        Parser::loadDetectionResult(detect_res_path.str().c_str(), frames, n_cams);
        std::cout << "  - " << frames.size() << " vaild frames loaded" << std::endl;
        
        std::stringstream world_point_path;
        world_point_path << output_dir << OS_SEP << "world_points" << OS_SEP << "world_points_initial.json";
        Parser::loadInitialCheckerboardPoses(world_point_path.str().c_str(), frames, chb_poses);
        std::cout << "  - " << chb_poses.size() << " chb poses loaded" << std::endl;
    }


    void run(const char* dir) {

        // loss function
        ceres::LossFunction *loss = NULL; 

        int n_cams = cameras.size();
        ceres::Problem ceres_prob;
        for (int frame_idx = 0; frame_idx < frames.size(); frame_idx++) {
            Frame *frame = &frames[frame_idx];
            std::array<double, 6> chb_pose = chb_poses[frame_idx];
            std::cout << frame_idx << "/" << frames.size() << " " << frame->img_name << std::endl;

            for(int cam_idx = 0; cam_idx < n_cams; cam_idx++ ){
                if(frame->detected[cam_idx]) {
                    Camera *cam = &cameras[cam_idx];
                    double *chb_rvec = &chb_pose[0];
                    double *chb_tvec = &chb_pose[3];
                    double *cam_param = &cam->params[0];
                    
                    // image point path
                    std::stringstream imgpts_path;
                    imgpts_path << ROOT_DIR << OS_SEP << "output" << OS_SEP << "corners" << OS_SEP << "cam_" << std::to_string(cam_idx) << OS_SEP << std::to_string(cam_idx) << "_" << frame->img_name.c_str() << ".txt";

                    ceres::CostFunction *cost_func = BundAdj6Dof::CreateReprojectionError(imgpts_path.str().c_str(), checkerboard, frame->n_detected);
                    ceres_prob.AddResidualBlock(cost_func, loss, cam->params, chb_rvec, chb_tvec);
                }
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = 1;
        options.max_num_iterations = 1000;
        options.num_threads = 4;
        options.function_tolerance = 1e-17;
        options.parameter_tolerance = 1e-17;
        options.gradient_tolerance = 1e-17;
        options.inner_iteration_tolerance = 1e-17;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &ceres_prob, &summary);
        std::cout << summary.FullReport() << std::endl;
        std::cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << std::endl;






    }

    static ceres::CostFunction* CreateReprojectionError(const char* imgpts_path_, Checkerboard *chb_, const int n_cams_used_) {
        constexpr int kChbRvecDim = 3;
        constexpr int kChbTvecDim = 3;
        constexpr int kNumResiduals = 2;
        ReprojectionError *re = new BundAdj6Dof::ReprojectionError(imgpts_path_, chb_, n_cams_used_);
        return (new ceres::AutoDiffCostFunction<ReprojectionError, kNumResiduals, NUM_CAM_PARAMS, kChbRvecDim, kChbTvecDim>(re));
    }
    struct ReprojectionError {
        Checkerboard *chb;
        double *img_pts;
        int n_cams_used;

        ReprojectionError(const char* imgpts_path_, Checkerboard *chb_, const int n_cams_used_){
            n_cams_used = n_cams_used_;
            chb = chb_;
            img_pts = new double[2*chb_->n_pts];
            int res = Parser::loadImagePoints(imgpts_path_, img_pts);
            assert(res == 0);
        }

        ~ReprojectionError() {
            delete[] img_pts;
        }


		template<typename T>
		bool operator()(const T* const camera, const T* const chbR, const T* const chbT, T* residuals) const {
            // residuals
            residuals[0] = T(0);
            residuals[1] = T(0);

            // camera parameters
			const T extrinsics_rvec[3] = { camera[0], camera[1], camera[2] };
			const T extrinsics_tvec[3] = { camera[3], camera[4], camera[5] };
			const T f[2] = { camera[6], camera[7] };
			const T c[2] = { camera[8], camera[9] };
            const T k[3] = { camera[10], camera[11], camera[14]};
            const T p[2] = { camera[12], camera[13]};

            // checkerboard
            const T chb_ang_axis[3] = {chbR[0], chbR[1], chbR[2]};
            const T chb_trans[3] = {chbT[0], chbT[1], chbT[2]};

            for ( size_t i = 0; i < chb->n_pts; i++) {
                // update checkerboard points (6 dof)
                const T chb_point[3] = {T(chb->chb_pts[i * 3]), T(chb->chb_pts[i*3 + 1]), T(chb->chb_pts[i*3+2])};
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

                // projection to image
                T x_pred = f[0]*u + c[0];
                T y_pred = f[1]*v + c[1];

                // reprojection error
                T dx = x_pred - img_pts[i*2];
                T dy = y_pred - img_pts[i*2 + 1];

                // output
                residuals[0] += dx / T(n_cams_used);
                residuals[1] += dy / T(n_cams_used);
            }
            return true;
        }
    };
};