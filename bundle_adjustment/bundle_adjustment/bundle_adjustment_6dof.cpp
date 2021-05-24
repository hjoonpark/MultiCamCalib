#define _USE_MATH_DEFINES
#include <math.h>

#include "bundle_adjustment_6dof.h"
#include "parser.h"
#define NUM_CAMPARAMS 22 // 3 + 3 + 2 + 2 + 6 + 6
#define NUM_CORNERS 24*17


class LossFunction {
public:
	virtual void Evaluate(double s, double out[3]) const = 0;
};

class BundleAdjustmentProblem6Dof {
public:
	struct CameraRegularization {
		Configs* configs;
		CameraRegularization(Configs* cf) {
			configs = cf;
		}
		static ceres::CostFunction* Create(Configs* cf) {
			// size of input parameters to 'bool operator()'
			static const int kResidualDim = 8; // 6 for radial, 2 for tangential
			const int kNumCamParam = NUM_CAMPARAMS;
			CameraRegularization* re = new BundleAdjustmentProblem6Dof::CameraRegularization(cf);
			return (new ceres::AutoDiffCostFunction<CameraRegularization, kResidualDim, kNumCamParam>(re));
		}

		template<typename T>
		bool operator()(const T* const camera, T* residuals) const {
			T k[6] = { T(0) };
			T p[6] = { T(0) };
			if (configs->radial_model == Configs::RadialModel::polynomial) {
				switch (configs->max_k) {
				case 6:
					k[5] = camera[15];
				case 5:
					k[4] = camera[14];
				case 4:
					k[3] = camera[13];
				case 3:
					k[2] = camera[12];
				case 2:
					k[1] = camera[11];
				case 1:
					k[0] = camera[10];
				}
			}

			else if (configs->radial_model == Configs::RadialModel::rational) {
				k[0] = camera[10];
				k[1] = camera[11];
				k[2] = camera[12];
				k[3] = camera[13];
				k[4] = camera[14];
				k[5] = camera[15];
			}

			switch (configs->max_p) {
			case 6:
				p[5] = camera[21];
			case 5:
				p[4] = camera[20];
			case 4:
				p[3] = camera[19];
			case 3:
				p[2] = camera[18];
			case 2:
				p[1] = camera[17];
			case 1:
				p[0] = camera[16];
			}

			// distortion coeffecient regularization
			static const double lambda_k[6] = { 1, 1, 1, 1, 1, 1 };
			static const double lambda_p[2] = { 1, 1 };
			if (configs->dist_regularization) {
				residuals[0] = lambda_k[0] * k[0];
				residuals[1] = lambda_k[1] * k[1];
				residuals[2] = lambda_k[2] * k[2];
				residuals[3] = lambda_k[3] * k[3];
				residuals[4] = lambda_k[4] * k[4];
				residuals[5] = lambda_k[5] * k[5];
				residuals[6] = lambda_p[0] * p[0];
				residuals[7] = lambda_p[1] * p[1];
			}
			else {
				residuals[0] = T(0);
				residuals[1] = T(0);
				residuals[2] = T(0);
				residuals[3] = T(0);
				residuals[4] = T(0);
				residuals[5] = T(0);
				residuals[6] = T(0);
				residuals[7] = T(0);
			}

			return true;
		}
	};
	struct ReprojectionError {
		static double chb_points[NUM_CORNERS * 3];
		double *image_points_mea;
		Configs *configs;
		double num_detected_cams;
		~ReprojectionError() {
			delete[] image_points_mea;
		}

		ReprojectionError(Configs *cf, const double num_detected_cams_, const int num_pts, double *observed_points) {
			configs = cf;
			image_points_mea = new double[num_pts * 2];
			num_detected_cams = num_detected_cams_;
			// observed_points -> 2*88 size
			for (int i = 0; i < num_pts; i++) {
				image_points_mea[i * 2 + 0] = observed_points[i * 2 + 0];
				image_points_mea[i * 2 + 1] = observed_points[i * 2 + 1];
			}
		}

		static ceres::CostFunction* Create(Configs *cf, const double num_detected_cams_, double *observed_points) {
			static const int kNumCamParam = NUM_CAMPARAMS;
			static const int kChbRvecDim = 3;
			static const int kChbTvecDim = 3;

			ReprojectionError *re = new BundleAdjustmentProblem6Dof::ReprojectionError(cf, num_detected_cams_, NUM_CORNERS, observed_points);
			return (new ceres::AutoDiffCostFunction<ReprojectionError, (NUM_CORNERS * 2), kNumCamParam, kChbRvecDim, kChbTvecDim>(re));
		}

		template<typename T>
		bool operator()(const T* const camera, const T* const chbR, const T* const chbT, T* residuals) const {
			// extract camera parameters
			const T ang_axis[3] = { camera[0], camera[1], camera[2] };
			const T trans[3] = { camera[3], camera[4], camera[5] };
			const T f[2] = { camera[6], camera[7] };
			const T c[2] = { camera[8], camera[9] };


			T k[6] = { T(0) };
			T p[6] = { T(0) };
			if (configs->radial_model == Configs::RadialModel::polynomial) {
				switch (configs->max_k) {
				case 6:
					k[5] = camera[15];
				case 5:
					k[4] = camera[14];
				case 4:
					k[3] = camera[13];
				case 3:
					k[2] = camera[12];
				case 2:
					k[1] = camera[11];
				case 1:
					k[0] = camera[10];
				}
			}

			else if (configs->radial_model == Configs::RadialModel::rational) {
				k[0] = camera[10];
				k[1] = camera[11];
				k[2] = camera[12];
				k[3] = camera[13];
				k[4] = camera[14];
				k[5] = camera[15];
			}

			switch (configs->max_p) {
			case 6:
				p[5] = camera[21];
			case 5:
				p[4] = camera[20];
			case 4:
				p[3] = camera[19];
			case 3:
				p[2] = camera[18];
			case 2:
				p[1] = camera[17];
			case 1:
				p[0] = camera[16];
			}

			// chb parameters
			const T chb_ang_axis[3] = { chbR[0], chbR[1], chbR[2] };
			const T chb_trans[3] = { chbT[0], chbT[1], chbT[2] };

			// world_points
			for (int i = 0; i < configs->chb_num_corners; i++) {
				// update checkerboard points
				const T chb_point[3] = { T(chb_points[i * 3]), T(chb_points[i * 3 + 1]), T(chb_points[i * 3 + 2]) };
				T world_point[3];
				ceres::AngleAxisRotatePoint(chb_ang_axis, chb_point, world_point);

				// translation
				world_point[0] += chb_trans[0];
				world_point[1] += chb_trans[1];
				world_point[2] += chb_trans[2];

				T cam_point[3];
				// angle-axis rotation
				ceres::AngleAxisRotatePoint(ang_axis, world_point, cam_point);

				// translation
				cam_point[0] += trans[0];
				cam_point[1] += trans[1];
				cam_point[2] += trans[2];

				// center of distortion
				T xp = cam_point[0] / cam_point[2];
				T yp = cam_point[1] / cam_point[2];

				T radial_dist = T(0);
				T r2 = T(0);
				if (configs->radial_model == Configs::RadialModel::polynomial) {
					// radial distortions
					r2 = xp * xp + yp * yp;
					T r2_radial = T(1.0);
					radial_dist = T(1.0);
					for (int kn = 0; kn < configs->max_k; kn++) {
						r2_radial *= r2;
						radial_dist += k[kn] * r2_radial;
					}
				}
				else if (configs->radial_model == Configs::RadialModel::rational) {
					// radial distortions
					r2 = xp * xp + yp * yp;
					T r4 = r2 * r2;
					T r6 = r2 * r2 * r2;
					radial_dist = (1.0 + k[0] * r2 + k[1] * r4 + k[2] * r6) / (1.0 + k[3] * r2 + k[4] * r4 + k[5] * r6);
				}

				// tangential distortions
				T tan_post = T(1.0);
				T r2_tangential = T(1.0);
				for (int pn = 2; pn < configs->max_p; pn++) {
					r2_tangential *= r2;
					tan_post += p[pn] * r2_tangential;
				}

				T tangential_dist_x = (p[1] * (r2 + 2.0 * xp*xp) + 2.0 * p[0] * xp*yp) * tan_post;
				T tangential_dist_y = (p[0] * (r2 + 2.0 * yp*yp) + 2.0 * p[1] * xp*yp) * tan_post;

				T u = xp * radial_dist + tangential_dist_x;
				T v = yp * radial_dist + tangential_dist_y;

				// projected point position
				T predicted_x = f[0] * u + c[0];
				T predicted_y = f[1] * v + c[1];

				// error
				T dx = predicted_x - image_points_mea[i * 2 + 0];
				T dy = predicted_y - image_points_mea[i * 2 + 1];

				// output
				residuals[2 * i + 0] = dx / T(num_detected_cams);
				residuals[2 * i + 1] = dy / T(num_detected_cams);
			}

			return true;
		}
	};

private:
	BundleAdjParameters params;
	Configs configs_;
	bool *detected_;
	double *image_points_;
	std::vector<std::string> img_names_vec;

public:
	Configs* GetConfigs() {
		return &configs_;
	}
	bool detected(const int frame_idx, const int cam_idx) {
		return detected_[frame_idx * configs_.num_cams + cam_idx];
	}
	double* image_points(const int frame_idx, const int cam_idx) {
		return image_points_ + frame_idx * configs_.num_cams*configs_.chb_num_corners*2 + cam_idx * configs_.chb_num_corners*2;
	}
	~BundleAdjustmentProblem6Dof() {
		delete[] detected_;
		delete[] image_points_;
	}
	double* chb_rvec(const int frame_idx) {
		return params.chb_rvecs_ + frame_idx * 3;
	}
	double* chb_tvec(const int frame_idx) {
		return params.chb_tvecs_ + frame_idx * 3;
	}
	double *world_points(const int frame_idx) {
		return params.world_points_ + frame_idx * configs_.chb_num_corners * 3;
	}
	double* cam_params(const int cam_idx) {
		return params.cam_params_ + cam_idx * configs_.num_cam_params;
	}

	std::string image_name(const int frame_idx) {
		return img_names_vec[frame_idx];
	}
	int Load(const char* img_pts_json, const char* initial_params_json) {
		Parser parser;
		parser.LoadImagePoints(img_pts_json, configs_, detected_, image_points_, img_names_vec, NUM_CORNERS);
		parser.LoadInitialParameters(initial_params_json, img_names_vec, configs_, params);
		return 0;
	}

	/*
	int GenerateInitialWorldPoints() {
		int num_frames = configs_.num_frames;
		int rows = configs_.chb_num_rows;
		int cols = configs_.chb_num_cols;
		int num_corners = configs_.chb_num_corners;
		double s = configs_.chb_sqr_size;

		double *wps = new double[num_frames * num_corners * 3];
		double *rvecs = new double[num_frames * 3];
		double *tvecs = new double[num_frames * 3];
		for (int frame_idx = 0; frame_idx < num_frames; frame_idx++) {
			for (int r = 0; r < rows; r++) {
				for (int c = 0; c < cols; c++) {
					double x = -c * s;
					double y = r * s;
					double z = 0;
					int p_idx = (frame_idx * num_corners * 3) + ((r * cols + c) * 3);
					wps[p_idx + 0] = x;
					wps[p_idx + 1] = y;
					wps[p_idx + 2] = z;
				}
			}

			int chb_idx = frame_idx * 3;
			rvecs[chb_idx + 0] = -M_PI * 0.5;
			rvecs[chb_idx + 1] = 0;
			rvecs[chb_idx + 2] = 0;

			tvecs[chb_idx + 0] = 0;
			tvecs[chb_idx + 1] = 0;
			tvecs[chb_idx + 2] = 1000;
		}
		params.SetChbParams(num_frames, num_corners, wps, rvecs, tvecs);

		delete[] wps, tvecs, rvecs;
		return 0;
	}
	*/
};

#define REAL_IMGS
#ifdef REAL_IMGS
# if 0
double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = { 
-0, 0, 0,
-60, 0, 0,
-120, 0, 0,
-180, 0, 0,
-240, 0, 0,
-300, 0, 0,
-360, 0, 0,
-420, 0, 0,
-480, 0, 0,
-540, 0, 0,
-600, 0, 0,
-0, 60, 0,
-60, 60, 0,
-120, 60, 0,
-180, 60, 0,
-240, 60, 0,
-300, 60, 0,
-360, 60, 0,
-420, 60, 0,
-480, 60, 0,
-540, 60, 0,
-600, 60, 0,
-0, 120, 0,
-60, 120, 0,
-120, 120, 0,
-180, 120, 0,
-240, 120, 0,
-300, 120, 0,
-360, 120, 0,
-420, 120, 0,
-480, 120, 0,
-540, 120, 0,
-600, 120, 0,
-0, 180, 0,
-60, 180, 0,
-120, 180, 0,
-180, 180, 0,
-240, 180, 0,
-300, 180, 0,
-360, 180, 0,
-420, 180, 0,
-480, 180, 0,
-540, 180, 0,
-600, 180, 0,
-0, 240, 0,
-60, 240, 0,
-120, 240, 0,
-180, 240, 0,
-240, 240, 0,
-300, 240, 0,
-360, 240, 0,
-420, 240, 0,
-480, 240, 0,
-540, 240, 0,
-600, 240, 0,
-0, 300, 0,
-60, 300, 0,
-120, 300, 0,
-180, 300, 0,
-240, 300, 0,
-300, 300, 0,
-360, 300, 0,
-420, 300, 0,
-480, 300, 0,
-540, 300, 0,
-600, 300, 0,
-0, 360, 0,
-60, 360, 0,
-120, 360, 0,
-180, 360, 0,
-240, 360, 0,
-300, 360, 0,
-360, 360, 0,
-420, 360, 0,
-480, 360, 0,
-540, 360, 0,
-600, 360, 0,
-0, 420, 0,
-60, 420, 0,
-120, 420, 0,
-180, 420, 0,
-240, 420, 0,
-300, 420, 0,
-360, 420, 0,
-420, 420, 0,
-480, 420, 0,
-540, 420, 0,
-600, 420, 0, };
#else

double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = {
0.0, 0.0, 0.0,
-30.0, 0.0, 0.0,
-60.0, 0.0, 0.0,
-90.0, 0.0, 0.0,
-120.0, 0.0, 0.0,
-150.0, 0.0, 0.0,
-180.0, 0.0, 0.0,
-210.0, 0.0, 0.0,
-240.0, 0.0, 0.0,
-270.0, 0.0, 0.0,
-300.0, 0.0, 0.0,
-330.0, 0.0, 0.0,
-360.0, 0.0, 0.0,
-390.0, 0.0, 0.0,
-420.0, 0.0, 0.0,
-450.0, 0.0, 0.0,
-480.0, 0.0, 0.0,
-510.0, 0.0, 0.0,
-540.0, 0.0, 0.0,
-570.0, 0.0, 0.0,
-600.0, 0.0, 0.0,
-630.0, 0.0, 0.0,
-660.0, 0.0, 0.0,
-690.0, 0.0, 0.0,
0.0, 30.0, 0.0,
-30.0, 30.0, 0.0,
-60.0, 30.0, 0.0,
-90.0, 30.0, 0.0,
-120.0, 30.0, 0.0,
-150.0, 30.0, 0.0,
-180.0, 30.0, 0.0,
-210.0, 30.0, 0.0,
-240.0, 30.0, 0.0,
-270.0, 30.0, 0.0,
-300.0, 30.0, 0.0,
-330.0, 30.0, 0.0,
-360.0, 30.0, 0.0,
-390.0, 30.0, 0.0,
-420.0, 30.0, 0.0,
-450.0, 30.0, 0.0,
-480.0, 30.0, 0.0,
-510.0, 30.0, 0.0,
-540.0, 30.0, 0.0,
-570.0, 30.0, 0.0,
-600.0, 30.0, 0.0,
-630.0, 30.0, 0.0,
-660.0, 30.0, 0.0,
-690.0, 30.0, 0.0,
0.0, 60.0, 0.0,
-30.0, 60.0, 0.0,
-60.0, 60.0, 0.0,
-90.0, 60.0, 0.0,
-120.0, 60.0, 0.0,
-150.0, 60.0, 0.0,
-180.0, 60.0, 0.0,
-210.0, 60.0, 0.0,
-240.0, 60.0, 0.0,
-270.0, 60.0, 0.0,
-300.0, 60.0, 0.0,
-330.0, 60.0, 0.0,
-360.0, 60.0, 0.0,
-390.0, 60.0, 0.0,
-420.0, 60.0, 0.0,
-450.0, 60.0, 0.0,
-480.0, 60.0, 0.0,
-510.0, 60.0, 0.0,
-540.0, 60.0, 0.0,
-570.0, 60.0, 0.0,
-600.0, 60.0, 0.0,
-630.0, 60.0, 0.0,
-660.0, 60.0, 0.0,
-690.0, 60.0, 0.0,
0.0, 90.0, 0.0,
-30.0, 90.0, 0.0,
-60.0, 90.0, 0.0,
-90.0, 90.0, 0.0,
-120.0, 90.0, 0.0,
-150.0, 90.0, 0.0,
-180.0, 90.0, 0.0,
-210.0, 90.0, 0.0,
-240.0, 90.0, 0.0,
-270.0, 90.0, 0.0,
-300.0, 90.0, 0.0,
-330.0, 90.0, 0.0,
-360.0, 90.0, 0.0,
-390.0, 90.0, 0.0,
-420.0, 90.0, 0.0,
-450.0, 90.0, 0.0,
-480.0, 90.0, 0.0,
-510.0, 90.0, 0.0,
-540.0, 90.0, 0.0,
-570.0, 90.0, 0.0,
-600.0, 90.0, 0.0,
-630.0, 90.0, 0.0,
-660.0, 90.0, 0.0,
-690.0, 90.0, 0.0,
0.0, 120.0, 0.0,
-30.0, 120.0, 0.0,
-60.0, 120.0, 0.0,
-90.0, 120.0, 0.0,
-120.0, 120.0, 0.0,
-150.0, 120.0, 0.0,
-180.0, 120.0, 0.0,
-210.0, 120.0, 0.0,
-240.0, 120.0, 0.0,
-270.0, 120.0, 0.0,
-300.0, 120.0, 0.0,
-330.0, 120.0, 0.0,
-360.0, 120.0, 0.0,
-390.0, 120.0, 0.0,
-420.0, 120.0, 0.0,
-450.0, 120.0, 0.0,
-480.0, 120.0, 0.0,
-510.0, 120.0, 0.0,
-540.0, 120.0, 0.0,
-570.0, 120.0, 0.0,
-600.0, 120.0, 0.0,
-630.0, 120.0, 0.0,
-660.0, 120.0, 0.0,
-690.0, 120.0, 0.0,
0.0, 150.0, 0.0,
-30.0, 150.0, 0.0,
-60.0, 150.0, 0.0,
-90.0, 150.0, 0.0,
-120.0, 150.0, 0.0,
-150.0, 150.0, 0.0,
-180.0, 150.0, 0.0,
-210.0, 150.0, 0.0,
-240.0, 150.0, 0.0,
-270.0, 150.0, 0.0,
-300.0, 150.0, 0.0,
-330.0, 150.0, 0.0,
-360.0, 150.0, 0.0,
-390.0, 150.0, 0.0,
-420.0, 150.0, 0.0,
-450.0, 150.0, 0.0,
-480.0, 150.0, 0.0,
-510.0, 150.0, 0.0,
-540.0, 150.0, 0.0,
-570.0, 150.0, 0.0,
-600.0, 150.0, 0.0,
-630.0, 150.0, 0.0,
-660.0, 150.0, 0.0,
-690.0, 150.0, 0.0,
0.0, 180.0, 0.0,
-30.0, 180.0, 0.0,
-60.0, 180.0, 0.0,
-90.0, 180.0, 0.0,
-120.0, 180.0, 0.0,
-150.0, 180.0, 0.0,
-180.0, 180.0, 0.0,
-210.0, 180.0, 0.0,
-240.0, 180.0, 0.0,
-270.0, 180.0, 0.0,
-300.0, 180.0, 0.0,
-330.0, 180.0, 0.0,
-360.0, 180.0, 0.0,
-390.0, 180.0, 0.0,
-420.0, 180.0, 0.0,
-450.0, 180.0, 0.0,
-480.0, 180.0, 0.0,
-510.0, 180.0, 0.0,
-540.0, 180.0, 0.0,
-570.0, 180.0, 0.0,
-600.0, 180.0, 0.0,
-630.0, 180.0, 0.0,
-660.0, 180.0, 0.0,
-690.0, 180.0, 0.0,
0.0, 210.0, 0.0,
-30.0, 210.0, 0.0,
-60.0, 210.0, 0.0,
-90.0, 210.0, 0.0,
-120.0, 210.0, 0.0,
-150.0, 210.0, 0.0,
-180.0, 210.0, 0.0,
-210.0, 210.0, 0.0,
-240.0, 210.0, 0.0,
-270.0, 210.0, 0.0,
-300.0, 210.0, 0.0,
-330.0, 210.0, 0.0,
-360.0, 210.0, 0.0,
-390.0, 210.0, 0.0,
-420.0, 210.0, 0.0,
-450.0, 210.0, 0.0,
-480.0, 210.0, 0.0,
-510.0, 210.0, 0.0,
-540.0, 210.0, 0.0,
-570.0, 210.0, 0.0,
-600.0, 210.0, 0.0,
-630.0, 210.0, 0.0,
-660.0, 210.0, 0.0,
-690.0, 210.0, 0.0,
0.0, 240.0, 0.0,
-30.0, 240.0, 0.0,
-60.0, 240.0, 0.0,
-90.0, 240.0, 0.0,
-120.0, 240.0, 0.0,
-150.0, 240.0, 0.0,
-180.0, 240.0, 0.0,
-210.0, 240.0, 0.0,
-240.0, 240.0, 0.0,
-270.0, 240.0, 0.0,
-300.0, 240.0, 0.0,
-330.0, 240.0, 0.0,
-360.0, 240.0, 0.0,
-390.0, 240.0, 0.0,
-420.0, 240.0, 0.0,
-450.0, 240.0, 0.0,
-480.0, 240.0, 0.0,
-510.0, 240.0, 0.0,
-540.0, 240.0, 0.0,
-570.0, 240.0, 0.0,
-600.0, 240.0, 0.0,
-630.0, 240.0, 0.0,
-660.0, 240.0, 0.0,
-690.0, 240.0, 0.0,
0.0, 270.0, 0.0,
-30.0, 270.0, 0.0,
-60.0, 270.0, 0.0,
-90.0, 270.0, 0.0,
-120.0, 270.0, 0.0,
-150.0, 270.0, 0.0,
-180.0, 270.0, 0.0,
-210.0, 270.0, 0.0,
-240.0, 270.0, 0.0,
-270.0, 270.0, 0.0,
-300.0, 270.0, 0.0,
-330.0, 270.0, 0.0,
-360.0, 270.0, 0.0,
-390.0, 270.0, 0.0,
-420.0, 270.0, 0.0,
-450.0, 270.0, 0.0,
-480.0, 270.0, 0.0,
-510.0, 270.0, 0.0,
-540.0, 270.0, 0.0,
-570.0, 270.0, 0.0,
-600.0, 270.0, 0.0,
-630.0, 270.0, 0.0,
-660.0, 270.0, 0.0,
-690.0, 270.0, 0.0,
0.0, 300.0, 0.0,
-30.0, 300.0, 0.0,
-60.0, 300.0, 0.0,
-90.0, 300.0, 0.0,
-120.0, 300.0, 0.0,
-150.0, 300.0, 0.0,
-180.0, 300.0, 0.0,
-210.0, 300.0, 0.0,
-240.0, 300.0, 0.0,
-270.0, 300.0, 0.0,
-300.0, 300.0, 0.0,
-330.0, 300.0, 0.0,
-360.0, 300.0, 0.0,
-390.0, 300.0, 0.0,
-420.0, 300.0, 0.0,
-450.0, 300.0, 0.0,
-480.0, 300.0, 0.0,
-510.0, 300.0, 0.0,
-540.0, 300.0, 0.0,
-570.0, 300.0, 0.0,
-600.0, 300.0, 0.0,
-630.0, 300.0, 0.0,
-660.0, 300.0, 0.0,
-690.0, 300.0, 0.0,
0.0, 330.0, 0.0,
-30.0, 330.0, 0.0,
-60.0, 330.0, 0.0,
-90.0, 330.0, 0.0,
-120.0, 330.0, 0.0,
-150.0, 330.0, 0.0,
-180.0, 330.0, 0.0,
-210.0, 330.0, 0.0,
-240.0, 330.0, 0.0,
-270.0, 330.0, 0.0,
-300.0, 330.0, 0.0,
-330.0, 330.0, 0.0,
-360.0, 330.0, 0.0,
-390.0, 330.0, 0.0,
-420.0, 330.0, 0.0,
-450.0, 330.0, 0.0,
-480.0, 330.0, 0.0,
-510.0, 330.0, 0.0,
-540.0, 330.0, 0.0,
-570.0, 330.0, 0.0,
-600.0, 330.0, 0.0,
-630.0, 330.0, 0.0,
-660.0, 330.0, 0.0,
-690.0, 330.0, 0.0,
0.0, 360.0, 0.0,
-30.0, 360.0, 0.0,
-60.0, 360.0, 0.0,
-90.0, 360.0, 0.0,
-120.0, 360.0, 0.0,
-150.0, 360.0, 0.0,
-180.0, 360.0, 0.0,
-210.0, 360.0, 0.0,
-240.0, 360.0, 0.0,
-270.0, 360.0, 0.0,
-300.0, 360.0, 0.0,
-330.0, 360.0, 0.0,
-360.0, 360.0, 0.0,
-390.0, 360.0, 0.0,
-420.0, 360.0, 0.0,
-450.0, 360.0, 0.0,
-480.0, 360.0, 0.0,
-510.0, 360.0, 0.0,
-540.0, 360.0, 0.0,
-570.0, 360.0, 0.0,
-600.0, 360.0, 0.0,
-630.0, 360.0, 0.0,
-660.0, 360.0, 0.0,
-690.0, 360.0, 0.0,
0.0, 390.0, 0.0,
-30.0, 390.0, 0.0,
-60.0, 390.0, 0.0,
-90.0, 390.0, 0.0,
-120.0, 390.0, 0.0,
-150.0, 390.0, 0.0,
-180.0, 390.0, 0.0,
-210.0, 390.0, 0.0,
-240.0, 390.0, 0.0,
-270.0, 390.0, 0.0,
-300.0, 390.0, 0.0,
-330.0, 390.0, 0.0,
-360.0, 390.0, 0.0,
-390.0, 390.0, 0.0,
-420.0, 390.0, 0.0,
-450.0, 390.0, 0.0,
-480.0, 390.0, 0.0,
-510.0, 390.0, 0.0,
-540.0, 390.0, 0.0,
-570.0, 390.0, 0.0,
-600.0, 390.0, 0.0,
-630.0, 390.0, 0.0,
-660.0, 390.0, 0.0,
-690.0, 390.0, 0.0,
0.0, 420.0, 0.0,
-30.0, 420.0, 0.0,
-60.0, 420.0, 0.0,
-90.0, 420.0, 0.0,
-120.0, 420.0, 0.0,
-150.0, 420.0, 0.0,
-180.0, 420.0, 0.0,
-210.0, 420.0, 0.0,
-240.0, 420.0, 0.0,
-270.0, 420.0, 0.0,
-300.0, 420.0, 0.0,
-330.0, 420.0, 0.0,
-360.0, 420.0, 0.0,
-390.0, 420.0, 0.0,
-420.0, 420.0, 0.0,
-450.0, 420.0, 0.0,
-480.0, 420.0, 0.0,
-510.0, 420.0, 0.0,
-540.0, 420.0, 0.0,
-570.0, 420.0, 0.0,
-600.0, 420.0, 0.0,
-630.0, 420.0, 0.0,
-660.0, 420.0, 0.0,
-690.0, 420.0, 0.0,
0.0, 450.0, 0.0,
-30.0, 450.0, 0.0,
-60.0, 450.0, 0.0,
-90.0, 450.0, 0.0,
-120.0, 450.0, 0.0,
-150.0, 450.0, 0.0,
-180.0, 450.0, 0.0,
-210.0, 450.0, 0.0,
-240.0, 450.0, 0.0,
-270.0, 450.0, 0.0,
-300.0, 450.0, 0.0,
-330.0, 450.0, 0.0,
-360.0, 450.0, 0.0,
-390.0, 450.0, 0.0,
-420.0, 450.0, 0.0,
-450.0, 450.0, 0.0,
-480.0, 450.0, 0.0,
-510.0, 450.0, 0.0,
-540.0, 450.0, 0.0,
-570.0, 450.0, 0.0,
-600.0, 450.0, 0.0,
-630.0, 450.0, 0.0,
-660.0, 450.0, 0.0,
-690.0, 450.0, 0.0,
0.0, 480.0, 0.0,
-30.0, 480.0, 0.0,
-60.0, 480.0, 0.0,
-90.0, 480.0, 0.0,
-120.0, 480.0, 0.0,
-150.0, 480.0, 0.0,
-180.0, 480.0, 0.0,
-210.0, 480.0, 0.0,
-240.0, 480.0, 0.0,
-270.0, 480.0, 0.0,
-300.0, 480.0, 0.0,
-330.0, 480.0, 0.0,
-360.0, 480.0, 0.0,
-390.0, 480.0, 0.0,
-420.0, 480.0, 0.0,
-450.0, 480.0, 0.0,
-480.0, 480.0, 0.0,
-510.0, 480.0, 0.0,
-540.0, 480.0, 0.0,
-570.0, 480.0, 0.0,
-600.0, 480.0, 0.0,
-630.0, 480.0, 0.0,
-660.0, 480.0, 0.0,
-690.0, 480.0, 0.0,
};
//
//double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = {
//300.0, -210.0, 0.0,
//240.0, -210.0, 0.0,
//180.0, -210.0, 0.0,
//120.0, -210.0, 0.0,
//60.0, -210.0, 0.0,
//-0.0, -210.0, 0.0,
//-60.0, -210.0, 0.0,
//-120.0, -210.0, 0.0,
//-180.0, -210.0, 0.0,
//-240.0, -210.0, 0.0,
//-300.0, -210.0, 0.0,
//300.0, -150.0, 0.0,
//240.0, -150.0, 0.0,
//180.0, -150.0, 0.0,
//120.0, -150.0, 0.0,
//60.0, -150.0, 0.0,
//
//-0.0, -150.0, 0.0,
//-60.0, -150.0, 0.0,
//-120.0, -150.0, 0.0,
//-180.0, -150.0, 0.0,
//-240.0, -150.0, 0.0,
//-300.0, -150.0, 0.0,
//300.0, -90.0, 0.0,
//240.0, -90.0, 0.0,
//180.0, -90.0, 0.0,
//120.0, -90.0, 0.0,
//60.0, -90.0, 0.0,
//-0.0, -90.0, 0.0,
//-60.0, -90.0, 0.0,
//-120.0, -90.0, 0.0,
//-180.0, -90.0, 0.0,
//-240.0, -90.0, 0.0,
//-300.0, -90.0, 0.0,
//300.0, -30.0, 0.0,
//240.0, -30.0, 0.0,
//180.0, -30.0, 0.0,
//120.0, -30.0, 0.0,
//60.0, -30.0, 0.0,
//-0.0, -30.0, 0.0,
//-60.0, -30.0, 0.0,
//-120.0, -30.0, 0.0,
//-180.0, -30.0, 0.0,
//-240.0, -30.0, 0.0,
//-300.0, -30.0, 0.0,
//300.0, 30.0, 0.0,
//240.0, 30.0, 0.0,
//180.0, 30.0, 0.0,
//120.0, 30.0, 0.0,
//60.0, 30.0, 0.0,
//-0.0, 30.0, 0.0,
//-60.0, 30.0, 0.0,
//-120.0, 30.0, 0.0,
//-180.0, 30.0, 0.0,
//-240.0, 30.0, 0.0,
//-300.0, 30.0, 0.0,
//300.0, 90.0, 0.0,
//240.0, 90.0, 0.0,
//180.0, 90.0, 0.0,
//120.0, 90.0, 0.0,
//60.0, 90.0, 0.0,
//-0.0, 90.0, 0.0,
//-60.0, 90.0, 0.0,
//-120.0, 90.0, 0.0,
//-180.0, 90.0, 0.0,
//-240.0, 90.0, 0.0,
//-300.0, 90.0, 0.0,
//300.0, 150.0, 0.0,
//240.0, 150.0, 0.0,
//180.0, 150.0, 0.0,
//120.0, 150.0, 0.0,
//60.0, 150.0, 0.0,
//-0.0, 150.0, 0.0,
//-60.0, 150.0, 0.0,
//-120.0, 150.0, 0.0,
//-180.0, 150.0, 0.0,
//-240.0, 150.0, 0.0,
//-300.0, 150.0, 0.0,
//300.0, 210.0, 0.0,
//240.0, 210.0, 0.0,
//180.0, 210.0, 0.0,
//120.0, 210.0, 0.0,
//60.0, 210.0, 0.0,
//-0.0, 210.0, 0.0,
//-60.0, 210.0, 0.0,
//-120.0, 210.0, 0.0,
//-180.0, 210.0, 0.0,
//-240.0, 210.0, 0.0,
//-300.0, 210.0, 0.0,
//};
#endif
#else

#define IS_FLAT 1
#if IS_FLAT
double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = {
-210.0, -300.0, 0.000,
-210.0, -240.0, 0.000,
-210.0, -180.0, 0.000,
-210.0, -120.0, 0.000,       
-210.0, -60.0, 0.000,
-210.0, 0.0, 0.000,
-210.0, 60.0, 0.000,
-210.0, 120.0, 0.000,
-210.0, 180.0, 0.000,
-210.0, 240.0, 0.000,
-210.0, 300.0, 0.000,
-150.0, -300.0, 0.000,
-150.0, -240.0, 0.000,
-150.0, -180.0, 0.000,
-150.0, -120.0, 0.000,
-150.0, -60.0, 0.000,
-150.0, 0.0, 0.000,
-150.0, 60.0, 0.000,
-150.0, 120.0, 0.000,
-150.0, 180.0, 0.000,
-150.0, 240.0, 0.000,
-150.0, 300.0, 0.000,
-90.0, -300.0, 0.000,
-90.0, -240.0, 0.000,
-90.0, -180.0, 0.000,
-90.0, -120.0, 0.000,
-90.0, -60.0, 0.000,
-90.0, 0.0, 0.000,
-90.0, 60.0, 0.000,
-90.0, 120.0, 0.000,
-90.0, 180.0, 0.000,
-90.0, 240.0, 0.000,
-90.0, 300.0, 0.000,
-30.0, -300.0, 0.000,
-30.0, -240.0, 0.000,
-30.0, -180.0, 0.000,
-30.0, -120.0, 0.000,
-30.0, -60.0, 0.000,
-30.0, 0.0, 0.000,
-30.0, 60.0, 0.000,
-30.0, 120.0, 0.000,
-30.0, 180.0, 0.000,
-30.0, 240.0, 0.000,
-30.0, 300.0, 0.000,
30.0, -300.0, 0.000,
30.0, -240.0, 0.000,
30.0, -180.0, 0.000,
30.0, -120.0, 0.000,
30.0, -60.0, 0.000,
30.0, 0.0, 0.000,
30.0, 60.0, 0.000,
30.0, 120.0, 0.000,
30.0, 180.0, 0.000,
30.0, 240.0, 0.000,
30.0, 300.0, 0.000,
90.0, -300.0, 0.000,
90.0, -240.0, 0.000,
90.0, -180.0, 0.000,
90.0, -120.0, 0.000,
90.0, -60.0, 0.000,
90.0, 0.0, 0.000,
90.0, 60.0, 0.000,
90.0, 120.0, 0.000,
90.0, 180.0, 0.000,
90.0, 240.0, 0.000,
90.0, 300.0, 0.000,
150.0, -300.0, 0.000,
150.0, -240.0, 0.000,
150.0, -180.0, 0.000,
150.0, -120.0, 0.000,
150.0, -60.0, 0.000,
150.0, 0.0, 0.000,
150.0, 60.0, 0.000,
150.0, 120.0, 0.000,
150.0, 180.0, 0.000,
150.0, 240.0, 0.000,
150.0, 300.0, 0.000,
210.0, -300.0, 0.000,
210.0, -240.0, 0.000,
210.0, -180.0, 0.000,
210.0, -120.0, 0.000,
210.0, -60.0, 0.000,
210.0, 0.0, 0.000,
210.0, 60.0, 0.000,
210.0, 120.0, 0.000,
210.0, 180.0, 0.000,
210.0, 240.0, 0.000,
210.0, 300.0, 0.000,
};
#else
double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = {
-210.0, -300.0, -1.365,
-210.0, -240.0, 0.432,
-210.0, -180.0, 1.873,
-210.0, -120.0, 2.927,
-210.0, -60.0, 3.569,
-210.0, 0.0, 3.785,
-210.0, 60.0, 3.569,
-210.0, 120.0, 2.927,
-210.0, 180.0, 1.873,
-210.0, 240.0, 0.432,
-210.0, 300.0, -1.365,
-150.0, -300.0, -1.365,
-150.0, -240.0, 0.432,
-150.0, -180.0, 1.873,
-150.0, -120.0, 2.927,
-150.0, -60.0, 3.569,
-150.0, 0.0, 3.785,
-150.0, 60.0, 3.569,
-150.0, 120.0, 2.927,
-150.0, 180.0, 1.873,
-150.0, 240.0, 0.432,
-150.0, 300.0, -1.365,
-90.0, -300.0, -1.365,
-90.0, -240.0, 0.432,
-90.0, -180.0, 1.873,
-90.0, -120.0, 2.927,
-90.0, -60.0, 3.569,
-90.0, 0.0, 3.785,
-90.0, 60.0, 3.569,
-90.0, 120.0, 2.927,
-90.0, 180.0, 1.873,
-90.0, 240.0, 0.432,
-90.0, 300.0, -1.365,
-30.0, -300.0, -1.365,
-30.0, -240.0, 0.432,
-30.0, -180.0, 1.873,
-30.0, -120.0, 2.927,
-30.0, -60.0, 3.569,
-30.0, 0.0, 3.785,
-30.0, 60.0, 3.569,
-30.0, 120.0, 2.927,
-30.0, 180.0, 1.873,
-30.0, 240.0, 0.432,
-30.0, 300.0, -1.365,
30.0, -300.0, -1.365,
30.0, -240.0, 0.432,
30.0, -180.0, 1.873,
30.0, -120.0, 2.927,
30.0, -60.0, 3.569,
30.0, 0.0, 3.785,
30.0, 60.0, 3.569,
30.0, 120.0, 2.927,
30.0, 180.0, 1.873,
30.0, 240.0, 0.432,
30.0, 300.0, -1.365,
90.0, -300.0, -1.365,
90.0, -240.0, 0.432,
90.0, -180.0, 1.873,
90.0, -120.0, 2.927,
90.0, -60.0, 3.569,
90.0, 0.0, 3.785,
90.0, 60.0, 3.569,
90.0, 120.0, 2.927,
90.0, 180.0, 1.873,
90.0, 240.0, 0.432,
90.0, 300.0, -1.365,
150.0, -300.0, -1.365,
150.0, -240.0, 0.432,
150.0, -180.0, 1.873,
150.0, -120.0, 2.927,
150.0, -60.0, 3.569,
150.0, 0.0, 3.785,
150.0, 60.0, 3.569,
150.0, 120.0, 2.927,
150.0, 180.0, 1.873,
150.0, 240.0, 0.432,
150.0, 300.0, -1.365,
210.0, -300.0, -1.365,
210.0, -240.0, 0.432,
210.0, -180.0, 1.873,
210.0, -120.0, 2.927,
210.0, -60.0, 3.569,
210.0, 0.0, 3.785,
210.0, 60.0, 3.569,
210.0, 120.0, 2.927,
210.0, 180.0, 1.873,
210.0, 240.0, 0.432,
210.0, 300.0, -1.365,
};
#endif
#endif

BundleAdjustment6Dof::BundleAdjustment6Dof(const int skip_every_frames_) {
	skip_every_frames = skip_every_frames_;
	cout << ">> skip_every_frames = " << skip_every_frames << endl;
}
int BundleAdjustment6Dof::DoWork(std::string work_path) {
	//std::string work_path = "D:/CalibrationData/CameraCalibration/2019_12_09_capture";
	cout << endl << ">> BundleAdjustment6Dof << " << endl << endl;
	google::InitGoogleLogging("HJP");

	// load input
	BundleAdjustmentProblem6Dof prob;
	Configs *configs = prob.GetConfigs();
	configs->input_image_pts_path = work_path + "/BundleAdjustment/input/image_points.json";
	configs->input_initial_params_path = work_path + "/BundleAdjustment/input/bund_adj_initial_params.json";
	prob.Load(configs->input_image_pts_path.c_str(), configs->input_initial_params_path.c_str());
	configs->max_k = 3; // 6
	configs->max_p = 2;
	configs->loss = Configs::Loss::None;
	configs->radial_model = Configs::RadialModel::polynomial; // rational
	configs->dist_regularization = true;
	int num_corners = configs->chb_num_corners;
	int num_frames = configs->num_frames;
	int num_cams = configs->num_cams;
	cout << "num_corners:" << num_corners << ", num_frames:" << num_frames << ", num_cams:" << num_cams << ", skip_every_frames:" << skip_every_frames << endl;

	std::string output_configs_path = work_path + "/BundleAdjustment/output/bundle_adjustment_6dof/configs.txt";
	configs->ExportConfigs(output_configs_path.c_str());

	// output txt
	std::ofstream output_txt;
	std::string out_path = work_path + "/BundleAdjustment/output/bundle_adjustment_6dof/bundleadjustment_output.txt";
	output_txt.open(out_path.c_str());
	// export initial camera parameters
	output_txt << num_frames << "\n";
	double* cp;
	for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
		cp = prob.cam_params(cam_idx);
		for (int i = 0; i < configs->num_cam_params; i++) {
			output_txt << cp[i] << " ";
		}
	}
	output_txt << "\n";

	// output json
	std::string json_out_path = work_path + "/BundleAdjustment/output/bundle_adjustment_6dof/bundleadjustment_output.json";
	rapidjson::StringBuffer writer_buf;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(writer_buf);
	writer.StartObject();
	
	writer.Key("configs");
	writer.StartObject();
	writer.Key("num_cams");
	writer.Uint(configs->num_cams);
	writer.Key("num_cam_params");
	writer.Uint(configs->num_cam_params);
	writer.Key("num_corners");
	writer.Uint(configs->chb_num_corners);
	writer.Key("num_frames");
	writer.Uint(configs->num_frames);
	writer.EndObject();
	
	writer.Key("cam_params_initial");
	writer.StartObject();
	for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
		double* cp = prob.cam_params(cam_idx);
		Parser::WriteCamParamsToJson(writer, cam_idx, cp);
	}
	writer.EndObject();

	// configure
	ceres::LossFunction* loss = NULL;
	if (configs->loss == Configs::Loss::Huber) {
		// delta: https://en.wikipedia.org/wiki/Huber_loss
		const double delta = 0.5;
		ceres::LossFunction* loss = new ceres::HuberLoss(delta);
	}



	// output json
	std::string json_out_path2 = work_path + "/BundleAdjustment/output/bundle_adjustment_6dof/input_check.json";
	rapidjson::StringBuffer writer_buf2;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer2(writer_buf2);
	writer2.StartObject();

	// ====================================== //
	// reprojection error blocks
	// ====================================== //
	ceres::Problem ceres_prob;
	int n_frames_used = 0;
	for (int frame_idx = 0; frame_idx < num_frames; frame_idx++) {
		/*
		if (frame_idx > num_frames * 0.5) {
			continue;
		}

		int remainder = frame_idx % skip_every_frames;
		if (remainder != 0) {
			continue;
		}
		*/
		//if (frame_idx > 1) break;

		writer2.Key(std::to_string(frame_idx).c_str());
		writer2.StartObject();

		int num_detected = 0;
		for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
			if (prob.detected(frame_idx, cam_idx)) {
				num_detected += 1;
			}
		}
		if (num_detected >= 2) {
			n_frames_used += 1;
			for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
				if (prob.detected(frame_idx, cam_idx)) {
					ceres::CostFunction* cost_func = BundleAdjustmentProblem6Dof::ReprojectionError::Create(prob.GetConfigs(), num_detected, prob.image_points(frame_idx, cam_idx));
					double *chb_rvec = prob.chb_rvec(frame_idx);
					double *chb_tvec = prob.chb_tvec(frame_idx);
					double *cam_params = prob.cam_params(cam_idx);

					writer2.Key(std::to_string(cam_idx).c_str());
					writer2.StartObject();
					writer2.Key("chb_rvec");
					writer2.StartArray();
					writer2.Double(chb_rvec[0]);
					writer2.Double(chb_rvec[1]);
					writer2.Double(chb_rvec[2]);
					writer2.EndArray();
					writer2.Key("chb_tvec");
					writer2.StartArray();
					writer2.Double(chb_tvec[0]);
					writer2.Double(chb_tvec[1]);
					writer2.Double(chb_tvec[2]);
					writer2.EndArray();
					writer2.EndObject();

					std::vector<double*> params;
					params.push_back(cam_params);
					params.push_back(chb_rvec);
					params.push_back(chb_tvec);

					ceres_prob.AddResidualBlock(cost_func, loss, cam_params, chb_rvec, chb_tvec);
					// ====================================== //
					// distortion coeff. regularization
					// ====================================== //
					// ceres::CostFunction* reg_func = BundleAdjustmentProblem6Dof::CameraRegularization::Create(prob.GetConfigs());
					// ceres_prob.AddResidualBlock(reg_func, loss, cam_params);
				}
			}
		}
		writer2.EndObject();
	}
	writer2.EndObject();

	rapidjson::Document json_out_doc2;
	json_out_doc2.Parse(writer_buf2.GetString());

	FILE* json_fp2;
	errno_t err2;
	err2 = fopen_s(&json_fp2, json_out_path2.c_str(), "wb"); // non-Windows use "w"
	char json_buf2[65536];
	rapidjson::FileWriteStream json_os2(json_fp2, json_buf2, sizeof(json_buf2));
	rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer_fs2(json_os2);
	json_out_doc2.Accept(writer_fs2);

	fclose(json_fp2);


	// ====================================== //
	// distortion coeff. regularization
	// ====================================== //
	/*
	for (int cam_idx = 0; cam_idx < num_cams; cam_idx++) {
		double* cam_params = prob.cam_params(cam_idx);
		ceres::CostFunction* cost_func = BundleAdjustmentProblem6Dof::CameraRegularization::Create(prob.GetConfigs());
		ceres_prob.AddResidualBlock(cost_func, loss, cam_params);
	}
	*/
	cout << n_frames_used << " frames used" << endl;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = 1;
	options.max_num_iterations = 1000;
	// options.num_linear_solver_threads = 4;
	options.num_threads = 4;
	options.function_tolerance = 1e-17;
	options.parameter_tolerance = 1e-17;
	options.gradient_tolerance = 1e-17;
	options.inner_iteration_tolerance = 1e-17;
	ceres::Solver::Summary summary;
	cout << "SOLVE" << endl;
	ceres::Solve(options, &ceres_prob, &summary);
	cout << "SOLVE" << endl;
	cout << summary.FullReport() << endl;
	//cout << "Estimated world point = (" << world_point[0] << ", " << world_point[1] << ", " << world_point[2] << ")" << endl;
	//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << endl;










	// some placeholders...
	for (int frame_idx = 0; frame_idx < configs->num_frames; frame_idx++) {
		output_txt << frame_idx << " " << prob.image_name(frame_idx) << " ";
		for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
			output_txt << prob.detected(frame_idx, cam_idx) << " ";
		}
		for (int world_point_idx = 0; world_point_idx < configs->chb_num_corners; world_point_idx++) {
			output_txt << -7 << " " << -7 << " " << -7 << " ";
		}
		output_txt << "\n";
	}

	// export final camera parameters
	for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
		cp = prob.cam_params(cam_idx);
		for (int i = 0; i < configs->num_cam_params; i++) {
			output_txt << cp[i] << " ";
		}
	}
	output_txt << "\n";
	std::vector<std::vector<std::array<double, 3>>> world_points;
	for (int frame_idx = 0; frame_idx < configs->num_frames; frame_idx++) {
		int num_detected = 0;
		for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
			num_detected += int(prob.detected(frame_idx, cam_idx));
		}

		output_txt << frame_idx << " " << prob.image_name(frame_idx) << " ";
		for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
			output_txt << prob.detected(frame_idx, cam_idx) << " ";
		}

		double *chb_rvec = prob.chb_rvec(frame_idx);
		double *chb_tvec = prob.chb_tvec(frame_idx);
		std::vector<std::array<double, 3>> wp_curr_frame;
		for (int p_idx = 0; p_idx < configs->chb_num_corners; p_idx++) {
			double chb_point[3] = { BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[p_idx * 3], BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[p_idx * 3 + 1], BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[p_idx * 3 + 2] };
			double world_point[3];
			ceres::AngleAxisRotatePoint(chb_rvec, chb_point, world_point);

			// translation
			world_point[0] += chb_tvec[0];
			world_point[1] += chb_tvec[1];
			world_point[2] += chb_tvec[2];
			output_txt << world_point[0] << " " << world_point[1] << " " << world_point[2] << " ";
			std::array<double, 3> world_pt = { world_point[0], world_point[1], world_point[2] };
			wp_curr_frame.push_back(world_pt);
		}
		world_points.push_back(wp_curr_frame);
		output_txt << "\n";
	}
	for (int frame_idx = 0; frame_idx < configs->num_frames; frame_idx++) {
		int num_detected = 0;
		for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
			num_detected += int(prob.detected(frame_idx, cam_idx));
		}

		output_txt << frame_idx << " " << prob.image_name(frame_idx) << " ";
		for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
			output_txt << prob.detected(frame_idx, cam_idx) << " ";
		}

		output_txt << prob.chb_rvec(frame_idx)[0] << " " << prob.chb_rvec(frame_idx)[1] << " " << prob.chb_rvec(frame_idx)[2] << " " << prob.chb_tvec(frame_idx)[0] << " " << prob.chb_tvec(frame_idx)[1] << " " << prob.chb_tvec(frame_idx)[2] << " ";
		output_txt << "\n";
	}

	output_txt << summary.initial_cost << " ";
	output_txt << summary.final_cost << " ";
	output_txt.close();


	writer.Key("final_loss");
	writer.Double(summary.final_cost);

	writer.Key("cam_params_final");
	writer.StartObject();
	for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
		double* cp = prob.cam_params(cam_idx);
		Parser::WriteCamParamsToJson(writer, cam_idx, cp);
	}
	writer.EndObject();

	writer.Key("chb");
	writer.StartObject();
	for (int frame_idx = 0; frame_idx < configs->num_frames; frame_idx++) {
		writer.Key(prob.image_name(frame_idx).c_str());

		writer.StartObject();
		writer.Key("world_pts");
		writer.StartArray();
		std::vector<std::array<double, 3>> wp_curr_frame = world_points[frame_idx];
		for (std::vector<std::array<double, 3>>::iterator it = std::begin(wp_curr_frame); it != std::end(wp_curr_frame); ++it) {
			std::array<double, 3> wp = *it;
			writer.StartArray();
			writer.Double(wp[0]);
			writer.Double(wp[1]);
			writer.Double(wp[2]);
			writer.EndArray();
		}
		writer.EndArray();
		writer.Key("frame_idx");
		writer.Uint(frame_idx);

		writer.Key("rvec");
		writer.StartArray();
		writer.Double(prob.chb_rvec(frame_idx)[0]);
		writer.Double(prob.chb_rvec(frame_idx)[1]);
		writer.Double(prob.chb_rvec(frame_idx)[2]);
		writer.EndArray();

		writer.Key("tvec");
		writer.StartArray();
		writer.Double(prob.chb_tvec(frame_idx)[0]);
		writer.Double(prob.chb_tvec(frame_idx)[1]);
		writer.Double(prob.chb_tvec(frame_idx)[2]);
		writer.EndArray();

		writer.EndObject();
	}
	writer.EndObject(); // writer.Key("chb");
	writer.EndObject();

	rapidjson::Document json_out_doc;
	json_out_doc.Parse(writer_buf.GetString());

	FILE* json_fp;
	errno_t err;
	err = fopen_s(&json_fp, json_out_path.c_str(), "wb"); // non-Windows use "w"
	char json_buf[65536];
	rapidjson::FileWriteStream json_os(json_fp, json_buf, sizeof(json_buf));
	rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer_fs(json_os);
	json_out_doc.Accept(writer_fs);

	fclose(json_fp);

	cout << endl << ">> Complete!" << endl;
	cout << "  frames used: " << n_frames_used << endl;
	cout << "  saved:" << out_path.c_str() << endl;
	cout << "  saved:" << json_out_path.c_str() << endl;


	return 0;
}

# if 0
#if 0
double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = { -0, 0, 0,
-60, 0, 0,
-120, 0, 0,
-180, 0, 0,
-240, 0, 0,
-300, 0, 0,
-360, 0, 0,
-420, 0, 0,
-480, 0, 0,
-540, 0, 0,
-600, 0, 0,
-0, 60, 0,
-60, 60, 0,
-120, 60, 0,
-180, 60, 0,
-240, 60, 0,
-300, 60, 0,
-360, 60, 0,
-420, 60, 0,
-480, 60, 0,
-540, 60, 0,
-600, 60, 0,
-0, 120, 0,
-60, 120, 0,
-120, 120, 0,
-180, 120, 0,
-240, 120, 0,
-300, 120, 0,
-360, 120, 0,
-420, 120, 0,
-480, 120, 0,
-540, 120, 0,
-600, 120, 0,
-0, 180, 0,
-60, 180, 0,
-120, 180, 0,
-180, 180, 0,
-240, 180, 0,
-300, 180, 0,
-360, 180, 0,
-420, 180, 0,
-480, 180, 0,
-540, 180, 0,
-600, 180, 0,
-0, 240, 0,
-60, 240, 0,
-120, 240, 0,
-180, 240, 0,
-240, 240, 0,
-300, 240, 0,
-360, 240, 0,
-420, 240, 0,
-480, 240, 0,
-540, 240, 0,
-600, 240, 0,
-0, 300, 0,
-60, 300, 0,
-120, 300, 0,
-180, 300, 0,
-240, 300, 0,
-300, 300, 0,
-360, 300, 0,
-420, 300, 0,
-480, 300, 0,
-540, 300, 0,
-600, 300, 0,
-0, 360, 0,
-60, 360, 0,
-120, 360, 0,
-180, 360, 0,
-240, 360, 0,
-300, 360, 0,
-360, 360, 0,
-420, 360, 0,
-480, 360, 0,
-540, 360, 0,
-600, 360, 0,
-0, 420, 0,
-60, 420, 0,
-120, 420, 0,
-180, 420, 0,
-240, 420, 0,
-300, 420, 0,
-360, 420, 0,
-420, 420, 0,
-480, 420, 0,
-540, 420, 0,
-600, 420, 0, };
#else
double BundleAdjustmentProblem6Dof::ReprojectionError::chb_points[NUM_CORNERS * 3] = {
-0.0000, 0.0000, -0.0000,
-60.0000, 0.0000, 0.7050,
-120.0000, 0.0000, 1.2820,
-180.0000, 0.0000, 1.7090,
-240.0000, 0.0000, 1.9720,
-300.0000, 0.0000, 2.0610,
-360.0000, 0.0000, 1.9720,
-420.0000, 0.0000, 1.7090,
-480.0000, 0.0000, 1.2820,
-540.0000, 0.0000, 0.7050,
-600.0000, 0.0000, -0.0000,
-0.0000, 60.0000, -0.0000,
-60.0000, 60.0000, 0.7050,
-120.0000, 60.0000, 1.2820,
-180.0000, 60.0000, 1.7090,
-240.0000, 60.0000, 1.9720,
-300.0000, 60.0000, 2.0610,
-360.0000, 60.0000, 1.9720,
-420.0000, 60.0000, 1.7090,
-480.0000, 60.0000, 1.2820,
-540.0000, 60.0000, 0.7050,
-600.0000, 60.0000, -0.0000,
-0.0000, 120.0000, -0.0000,
-60.0000, 120.0000, 0.7050,
-120.0000, 120.0000, 1.2820,
-180.0000, 120.0000, 1.7090,
-240.0000, 120.0000, 1.9720,
-300.0000, 120.0000, 2.0610,
-360.0000, 120.0000, 1.9720,
-420.0000, 120.0000, 1.7090,
-480.0000, 120.0000, 1.2820,
-540.0000, 120.0000, 0.7050,
-600.0000, 120.0000, -0.0000,
-0.0000, 180.0000, -0.0000,
-60.0000, 180.0000, 0.7050,
-120.0000, 180.0000, 1.2820,
-180.0000, 180.0000, 1.7090,
-240.0000, 180.0000, 1.9720,
-300.0000, 180.0000, 2.0610,
-360.0000, 180.0000, 1.9720,
-420.0000, 180.0000, 1.7090,
-480.0000, 180.0000, 1.2820,
-540.0000, 180.0000, 0.7050,
-600.0000, 180.0000, -0.0000,
-0.0000, 240.0000, -0.0000,
-60.0000, 240.0000, 0.7050,
-120.0000, 240.0000, 1.2820,
-180.0000, 240.0000, 1.7090,
-240.0000, 240.0000, 1.9720,
-300.0000, 240.0000, 2.0610,
-360.0000, 240.0000, 1.9720,
-420.0000, 240.0000, 1.7090,
-480.0000, 240.0000, 1.2820,
-540.0000, 240.0000, 0.7050,
-600.0000, 240.0000, -0.0000,
-0.0000, 300.0000, -0.0000,
-60.0000, 300.0000, 0.7050,
-120.0000, 300.0000, 1.2820,
-180.0000, 300.0000, 1.7090,
-240.0000, 300.0000, 1.9720,
-300.0000, 300.0000, 2.0610,
-360.0000, 300.0000, 1.9720,
-420.0000, 300.0000, 1.7090,
-480.0000, 300.0000, 1.2820,
-540.0000, 300.0000, 0.7050,
-600.0000, 300.0000, -0.0000,
-0.0000, 360.0000, -0.0000,
-60.0000, 360.0000, 0.7050,
-120.0000, 360.0000, 1.2820,
-180.0000, 360.0000, 1.7090,
-240.0000, 360.0000, 1.9720,
-300.0000, 360.0000, 2.0610,
-360.0000, 360.0000, 1.9720,
-420.0000, 360.0000, 1.7090,
-480.0000, 360.0000, 1.2820,
-540.0000, 360.0000, 0.7050,
-600.0000, 360.0000, -0.0000,
-0.0000, 420.0000, -0.0000,
-60.0000, 420.0000, 0.7050,
-120.0000, 420.0000, 1.2820,
-180.0000, 420.0000, 1.7090,
-240.0000, 420.0000, 1.9720,
-300.0000, 420.0000, 2.0610,
-360.0000, 420.0000, 1.9720,
-420.0000, 420.0000, 1.7090,
-480.0000, 420.0000, 1.2820,
-540.0000, 420.0000, 0.7050,
-600.0000, 420.0000, -0.0000,
};
#endif
#endif