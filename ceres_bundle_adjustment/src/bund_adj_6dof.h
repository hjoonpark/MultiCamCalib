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
    std::vector<Camera> cameras;
    std::vector<Frame> frames;
    std::vector<Checkerboard> checkerboards;
    BundAdj6Dof::Config config;
    void init(const char* output_dir, const Config &config_) {
        config = config_;
        Checkerboard::initGlobalProperties(config.chb_n_rows, config.chb_n_cols, config.chb_sqr_size);

        std::stringstream cam_path;
        cam_path << output_dir << OS_SEP << "cam_params" << OS_SEP << "cam_params_initial.json";
        std::cout << "Read: " << cam_path.str() << std::endl;
        Parser::loadInitialCamParams(cam_path.str().c_str(), config.n_cams, cameras);
        std::cout << "  - " << cameras.size() << " cameras loaded" << std::endl;

        std::stringstream detect_res_path;
        detect_res_path << output_dir << OS_SEP << "detection_result.json";
        std::stringstream outlier_path;
        outlier_path << output_dir << OS_SEP << "vae_outlier_detector" << OS_SEP << "outliers.json";
        Parser::loadDetectionResult(outlier_path.str().c_str(), detect_res_path.str().c_str(), frames, config.n_cams);
        std::cout << "  - " << frames.size() << " vaild frames loaded" << std::endl;
        
        std::stringstream world_point_path;
        world_point_path << output_dir << OS_SEP << "world_points" << OS_SEP << "world_points_initial.json";
        Parser::loadInitialCheckerboardPoses(world_point_path.str().c_str(), frames, checkerboards);
        std::cout << "  - " << checkerboards.size() << " chb poses loaded" << std::endl;
    }

    void run(const char* output_dir) {
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
                    std::stringstream imgpts_path;
                    imgpts_path << output_dir << OS_SEP << "corners" << OS_SEP << "cam_" << std::to_string(cam_idx) << OS_SEP << std::to_string(cam_idx) << "_" << frame->img_name.c_str() << ".txt";

                    // ceres::CostFunction *cost_func = BundAdj6Dof::ReprojectionError::Create(imgpts_path.str().c_str(), chb, frame->n_detected);
                    BundAdj6Dof::ReprojectionError::ReprojectionErrorCostFunc *cost_func = BundAdj6Dof::ReprojectionError::Create(imgpts_path.str().c_str(), chb, frame->n_detected);

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

        // save outputs
        std::stringstream cam_out_path;
        cam_out_path << output_dir << OS_SEP << "cam_params" << OS_SEP << "cam_params_final.json";
        rapidjson::StringBuffer writer_buf;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(writer_buf);
        writer.StartObject();
        for (int cam_idx = 0; cam_idx < n_cams; cam_idx++ ){
            Camera *cam = &cameras[cam_idx];
            writer.Key(std::to_string(cam_idx).c_str());
            writer.StartObject();
            {
                writer.Key("fx");
                writer.Double(cam->fx());
                writer.Key("fy");
                writer.Double(cam->fy());

                writer.Key("cx");
                writer.Double(cam->cx());
                writer.Key("cy");
                writer.Double(cam->cy());

                writer.Key("k1");
                writer.Double(cam->k1());
                writer.Key("k2");
                writer.Double(cam->k2());
                writer.Key("p1");
                writer.Double(cam->p1());
                writer.Key("p2");
                writer.Double(cam->p2());
                writer.Key("k3");
                writer.Double(cam->k3());

                writer.Key("rvec");
                writer.StartArray();
                writer.Double(cam->rvec()[0]);
                writer.Double(cam->rvec()[1]);
                writer.Double(cam->rvec()[2]);
                writer.EndArray();
                
                writer.Key("tvec");
                writer.StartArray();
                writer.Double(cam->tvec()[0]);
                writer.Double(cam->tvec()[1]);
                writer.Double(cam->tvec()[2]);
                writer.EndArray();
            }
            writer.EndObject();
        }
        writer.EndObject();
        
        Parser::writeJson(cam_out_path.str().c_str(), writer_buf);
        std::cout << ">> Camera parameters saved: " << cam_out_path.str().c_str() << std::endl;

        // save final world points
        std::stringstream worldpoints_out_path;
        worldpoints_out_path << output_dir << OS_SEP << "world_points" << OS_SEP << "world_points_final.json";
        rapidjson::StringBuffer writer_buf_wp;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer_wp(writer_buf_wp);
        writer_wp.StartObject();
        writer_wp.Key("checkerboard");
        writer_wp.StartObject();
        {
            writer_wp.Key("n_rows");
            writer_wp.Int(Checkerboard::n_rows);
            writer_wp.Key("n_cols");
            writer_wp.Int(Checkerboard::n_cols);
            writer_wp.Key("sqr_size");
            writer_wp.Double(Checkerboard::sqr_size);
        }
        writer_wp.EndObject();
        writer_wp.Key("frames");
        writer_wp.StartObject();
        for(int frame_idx = 0; frame_idx < checkerboards.size(); frame_idx++) {
            Checkerboard *chb = &checkerboards[frame_idx];
            std::vector<std::array<double, 3>> wps;
            for (int p_idx = 0; p_idx < Checkerboard::n_pts; p_idx++) {
                double wp0[3] = {Checkerboard::chb_pts[3*p_idx], Checkerboard::chb_pts[3*p_idx+1], Checkerboard::chb_pts[3*p_idx+2]};
                double wp1[3];
                ceres::AngleAxisRotatePoint(chb->rvec, wp0, wp1);
                wp1[0] += chb->tvec[0];
                wp1[1] += chb->tvec[1];
                wp1[2] += chb->tvec[2];
                std::array<double, 3> pt = {wp1[0], wp1[1], wp1[2]};
                wps.push_back(pt);
            }

            const char* img_name = chb->img_name.c_str(); 
            writer_wp.Key(img_name);
            writer_wp.StartObject();
            {
                writer_wp.Key("n_detected");
                writer_wp.Int(frames[frame_idx].n_detected);

                writer_wp.Key("rvec");
                writer_wp.StartArray();
                {
                    writer_wp.Double(chb->rvec[0]);
                    writer_wp.Double(chb->rvec[1]);
                    writer_wp.Double(chb->rvec[2]);
                }
                writer_wp.EndArray();

                writer_wp.Key("tvec");
                writer_wp.StartArray();
                {
                    writer_wp.Double(chb->tvec[0]);
                    writer_wp.Double(chb->tvec[1]);
                    writer_wp.Double(chb->tvec[2]);
                }
                writer_wp.EndArray();

                writer_wp.Key("world_pts");
                writer_wp.StartArray();
                for(int i = 0; i < Checkerboard::n_pts; i++) {
                    writer_wp.StartArray();
                    writer_wp.Double(wps[i][0]);
                    writer_wp.Double(wps[i][1]);
                    writer_wp.Double(wps[i][2]);
                    writer_wp.EndArray();
                }
                writer_wp.EndArray();
            }
            writer_wp.EndObject();
        }
        writer_wp.EndObject();
        writer_wp.EndObject();
        Parser::writeJson(worldpoints_out_path.str().c_str(), writer_buf_wp);

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
