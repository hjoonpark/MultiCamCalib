#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <stdio.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#define NUM_CAM_PARAMS 15
class LossFunction {
public:
	virtual void Evaluate(double s, double out[3]) const = 0;
};

struct Checkerboard {
    size_t n_rows, n_cols;
    double sqr_size;
    double *chb_pts;
    Checkerboard(size_t n_rows_, size_t n_cols_, double sqr_size_) {
        n_rows = n_rows_;
        n_cols = n_cols_;
        sqr_size = sqr_size_;
        
        /*
                columns
            +z o -------- +y
        rows|
            |
            | +x
        */
        chb_pts = new double[3*n_rows_*n_cols_];
        for (size_t r = 0; r < n_rows; r++ ) {
            for (size_t c = 0; c< n_cols; c++) {
                size_t i = r*n_cols + c;
                chb_pts[i] = r*sqr_size;
                chb_pts[i + 1] = c*sqr_size;
                chb_pts[i + 2] = 0;
            }
        }
    }
    ~Checkerboard() {
        delete[] chb_pts;
    }
};

class BundAdj6Dof_Problem {
public:
    struct ReprojectionError {
        Checkerboard *chb;
        double *img_pts;
        int n_cams_used;
        int n_chb_pts;

        ReprojectionError(const int n_cams_used_, const int n_chb_pts_, const double *img_pts_){
            n_cams_used = n_cams_used_;
            n_chb_pts = n_chb_pts_;
            std::memcpy(img_pts, img_pts_, sizeof(int)*n_chb_pts);
        }
        ~ReprojectionError() {
            delete[] img_pts;
        }

        static ceres::CostFunction* Create(const int n_cams_used_, const int n_chb_pts_, const double *img_pts_) {
            constexpr int kChbRvecDim = 3;
            constexpr int kChbTvecDim = 3;
            constexpr int kNumResiduals = 2;
            ReprojectionError *re = new BundAdj6Dof_Problem::ReprojectionError(n_cams_used_, n_chb_pts_, img_pts_);
            return (new ceres::AutoDiffCostFunction<ReprojectionError, kNumResiduals, NUM_CAM_PARAMS, kChbRvecDim, kChbTvecDim>(re));
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

            for ( size_t i = 0; i < n_chb_pts; i++) {
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

class BundAdj6Dof {
    void Run() {
        // load input
        BundAdj6Dof_Problem prob;
        
    }
};