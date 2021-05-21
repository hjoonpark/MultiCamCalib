#include "triangulation.h"
#include "parser.h"
#include <iostream>
#include <fstream>

#define NUM_CORNERS 408


class TriangulationProblem {
public:

	struct ReprojectionError {
		double u_mea, v_mea;
		Camera* cam;
		Configs* configs;

		~ReprojectionError() {

		}
		ReprojectionError(const double* uv, Camera* c, Configs* conf) {
			u_mea = uv[0];
			v_mea = uv[1];
			cam = c;
			configs = conf;
		}

		template<typename T>
		bool operator()(const T* const world_point, T* residuals) const {
			// extract camera parameters
			const T ang_axis[3] = { T(cam->rvec[0]), T(cam->rvec[1]), T(cam->rvec[2]) };
			const T trans[3] = { T(cam->tvec[0]), T(cam->tvec[1]), T(cam->tvec[2]) };
			const T f[2] = { T(cam->fx), T(cam->fy) };
			const T c[2] = { T(cam->cx), T(cam->cy) };

			T k[6] = { T(0) };
			T p[6] = { T(0) };
			for (int i = 0; i < 6; i++) {
				k[i] = T(cam->k[i]);
				p[i] = T(cam->p[i]);
			}

			// world_points
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

			T tangential_dist_x = (p[1] * (r2 + 2.0 * xp * xp) + 2.0 * p[0] * xp * yp) * tan_post;
			T tangential_dist_y = (p[0] * (r2 + 2.0 * yp * yp) + 2.0 * p[1] * xp * yp) * tan_post;

			T u = xp * radial_dist + tangential_dist_x;
			T v = yp * radial_dist + tangential_dist_y;

			// projected point position
			T u_pred = f[0] * u + c[0];
			T v_pred = f[1] * v + c[1];

			// error
			T du = u_pred - u_mea;
			T dv = v_pred - v_mea;

			// output
			residuals[0] = du;
			residuals[1] = dv;
			return true;
		}

		static ceres::CostFunction* Create(const double* uv, Camera* cp_in, Configs* conf) {
			ReprojectionError* re = new TriangulationProblem::ReprojectionError(uv, cp_in, conf);
			return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(re));
		}
	};

	std::map<std::string, int> imgname_2_frameidx;
	std::map<int, std::string> frameidx_2_imgname;

	~TriangulationProblem() {
		delete[] labels_, world_points_, image_points_, cam_indices_, obsv2worldpts_index_map;
	}
	Configs* configs() {
		return &configs_;
	}
	double* world_points(const int wp_idx) {
		return world_points_ + 3 * wp_idx;
	}
	double* image_points(const int obsv_idx) {
		return image_points_ + 2 * obsv_idx;
	}
	int camera_index(const int obsv_idx) {
		return cam_indices_[obsv_idx];
	}
	int obsv2worldpts_index(const int obsv_idx) {
		return obsv2worldpts_index_map[obsv_idx];
	}
	int world_pt_label(const int wp_idx) {
		return labels_[wp_idx];
	}
	Camera* camera(const int cam_idx) {
		return &cams_[cam_idx];
	}

	int LoadImagePointsAndWorldpoints(const char* path) {
		printf("Load image points from json: %s\n", path);
		FILE* fp = fopen(path, "rb"); // non-Windows use "r"

		char readBuffer[65536]; // max buffer size
		rapidjson::FileReadStream fs(fp, readBuffer, sizeof(readBuffer));

		// parse
		rapidjson::Document doc;
		doc.ParseStream(fs);
		assert(doc.HasMember("configs"));
		assert(doc.HasMember("image_points_list"));
		assert(doc.HasMember("world_points_initial"));

		// configs
		const rapidjson::Value& configs = doc["configs"];
		const int num_observations = configs["num_observations"].GetInt();
		const int num_world_points = configs["num_world_pts"].GetInt();
		configs_.num_cams = configs["num_cams"].GetInt();
		configs_.num_observations = num_observations;
		configs_.num_world_pts = num_world_points;

		// image points
		const rapidjson::Value& image_points_list = doc["image_points_list"];
		image_points_ = new double[num_observations * 2]{ 0 };
		cam_indices_ = new int[num_observations];
		obsv2worldpts_index_map = new int[num_observations];

		for (rapidjson::SizeType obsv_idx = 0; obsv_idx < num_observations; obsv_idx++) {
			const rapidjson::Value& data = image_points_list[obsv_idx];
			const double u = data["u"].GetDouble();
			const double v = data["v"].GetDouble();

			const int cam_idx = data["cam_idx"].GetInt();
			const int wp_idx = data["wp_idx"].GetInt();
			image_points_[obsv_idx * 2 + 0] = u;
			image_points_[obsv_idx * 2 + 1] = v;
			cam_indices_[obsv_idx] = cam_idx; 
			obsv2worldpts_index_map[obsv_idx] = wp_idx;
		}

		// initial world points
		const rapidjson::Value& world_pts = doc["world_points_initial"];
		world_points_ = new double[num_world_points * 3]{ 0 };
		labels_ = new int[num_world_points] {-1};

		for (rapidjson::SizeType wp_idx = 0; wp_idx < num_world_points; wp_idx++) {
			const rapidjson::Value& wp = world_pts[wp_idx];
			labels_[wp_idx] = wp["label"].GetInt();

			world_points_[wp_idx * 3 + 0] = wp["x"].GetDouble();
			world_points_[wp_idx * 3 + 1] = wp["y"].GetDouble();
			world_points_[wp_idx * 3 + 2] = wp["z"].GetDouble();
		}

		printf("  %d cameras, %d world points, %d observed image points loaded.\n", configs_.num_cams, num_world_points, num_observations);
		fclose(fp);
		return 0;
	}

	int LoadCameraParams(const char* path) {
		printf("Load json: %s\n", path);
		FILE* fp = fopen(path, "rb"); // non-Windows use "r"

		char readBuffer[65536]; // max buffer size
		rapidjson::FileReadStream fs(fp, readBuffer, sizeof(readBuffer));

		// parse
		rapidjson::Document doc;
		doc.ParseStream(fs);
		assert(doc.HasMember("configs"));
		assert(doc.HasMember("cam_params"));

		const rapidjson::Value& configs = doc["configs"];
		configs_.num_cams = configs["num_cams"].GetInt();

		const rapidjson::Value& cams = doc["cam_params"];
		for (int cam_idx = 0; cam_idx < configs_.num_cams; cam_idx++) {
			const rapidjson::Value& params = cams[std::to_string(cam_idx).c_str()];
			Camera cam(cam_idx);

			const rapidjson::Value& rvec = params["rvec"];
			cam.rvec[0] = rvec[0].GetDouble();
			cam.rvec[1] = rvec[1].GetDouble();
			cam.rvec[2] = rvec[2].GetDouble();
			const rapidjson::Value& tvec = params["tvec"];
			cam.tvec[0] = tvec[0].GetDouble();
			cam.tvec[1] = tvec[1].GetDouble();
			cam.tvec[2] = tvec[2].GetDouble();
			cam.fx = params["fx"].GetDouble();
			cam.fy = params["fy"].GetDouble();
			cam.cx = params["cx"].GetDouble();
			cam.cy = params["cy"].GetDouble();
			cam.k[0] = params["k1"].GetDouble();
			cam.k[1] = params["k2"].GetDouble();
			cam.k[2] = params["k3"].GetDouble();
			cam.k[3] = params["k4"].GetDouble();
			cam.k[4] = params["k5"].GetDouble();
			cam.k[5] = params["k6"].GetDouble();

			cam.p[0] = params["p1"].GetDouble();
			cam.p[1] = params["p2"].GetDouble();
			cam.p[2] = params["p3"].GetDouble();
			cam.p[3] = params["p4"].GetDouble();
			cam.p[4] = params["p5"].GetDouble();
			cam.p[5] = params["p6"].GetDouble();

			printf("Camera[%d]\n", cam_idx);
			printf("  (%.4f, %.4f, %.4f), (%.4f, %.4f, %.4f), (%.4f, %.4f), (%.4f, %.4f)\n", cam.rvec[0], cam.rvec[1], cam.rvec[2], cam.tvec[0], cam.tvec[1], cam.tvec[2], cam.fx, cam.fy, cam.cx, cam.cy);
			printf("  [k] %.4f, %.4f, %.4f, %.4f, %.4f, %.4f | p[p] %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", cam.k[0], cam.k[1], cam.k[2], cam.k[3], cam.k[4], cam.k[5], cam.p[0], cam.p[1], cam.p[2], cam.p[3], cam.p[4], cam.p[5]);
			cams_.push_back(cam);
		}
		printf("  %zd cameras loaded.\n", cams_.size());

		return 0;

	}
private:
	std::vector<Camera> cams_;
	Configs configs_;
	double* image_points_;
	int* obsv2worldpts_index_map; // observation index to world point index;
	int* cam_indices_; // corresponding to each image point

	double* world_points_;
	int* labels_; // "labelme" labels corresponding to each world point
};
Triangulation::Triangulation(std::string input_image_pt_path_) {
	input_image_pt_path = input_image_pt_path_;
}
#include <string>
#include <sstream>
#include <vector>
std::vector<std::string> split(const std::string& s, char delim) {
	std::stringstream ss(s);
	std::string item;
	std::vector<std::string> elems;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
		// elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
	}
	return elems;
}
int Triangulation::DoWork(std::string work_path) {
	google::InitGoogleLogging("HJP");

	TriangulationProblem prob;
	std::string input_img_pts = input_image_pt_path;
	prob.LoadImagePointsAndWorldpoints(input_img_pts.c_str());
	std::vector<std::string> tokens = split(input_img_pts, '-');


	std::string input_cam_params = work_path + std::string("/input/cam_params.json");
	prob.LoadCameraParams(input_cam_params.c_str());
	Configs* configs = prob.configs();
	configs->max_k = 3;
	configs->max_p = 2;
	configs->loss = Configs::Loss::None;
	configs->radial_model = Configs::RadialModel::polynomial;

	int num_cams = prob.configs()->num_cams;
	int num_observations = prob.configs()->num_observations;
	int num_world_pts = prob.configs()->num_world_pts;

	ceres::Problem ceres_prob;
	for (int obsv_idx = 0; obsv_idx < num_observations; obsv_idx++) {
		
		int remainder = obsv_idx % 100;
		if (remainder == 0) {
			printf("[%d]\n", obsv_idx);
		}
		ceres::LossFunction* loss = NULL;

		double* uv = prob.image_points(obsv_idx);
		const int cam_idx = prob.camera_index(obsv_idx);
		Camera* cam = prob.camera(cam_idx);

		const int wp_idx = prob.obsv2worldpts_index(obsv_idx);
		ceres::CostFunction* cost_function = TriangulationProblem::ReprojectionError::Create(uv, cam, configs);
		std::vector<double*> params;
		params.push_back(prob.world_points(wp_idx));
		ceres_prob.AddResidualBlock(cost_function, loss, params);
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.minimizer_progress_to_stdout = 1;
	options.max_num_iterations = 1000;

	//options.num_linear_solver_threads = 4;
	options.num_threads = 4;
	options.function_tolerance = 1e-15;
	options.gradient_tolerance = 1e-15;
	options.parameter_tolerance = 1e-15;
	options.inner_iteration_tolerance = 1e-15;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &ceres_prob, &summary);
	cout << summary.FullReport() << endl;
	cout << "Cost: " << summary.initial_cost << " -> " << summary.final_cost << ", iterations: " << summary.iterations.back().iteration << endl;

	// save triangulation results
	std::string json_out_path = work_path + std::string("/output/triangulation_output_") + tokens[1];
	rapidjson::StringBuffer writer_buf;
	rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(writer_buf);
	writer.StartObject();
	{
		writer.Key("configs");
		writer.StartObject();
		{
			writer.Key("num_world_pts");
			writer.Uint(configs->num_world_pts);
			writer.Key("num_observations");
			writer.Uint(configs->num_observations);
		}
		writer.EndObject();
		writer.Key("cam_params");
		writer.StartObject();
		for (int cam_idx = 0; cam_idx < configs->num_cams; cam_idx++) {
			Camera* cam = prob.camera(cam_idx);
			Parser::WriteCamParamsToJson(writer, cam_idx, cam);
		}
		writer.EndObject();

		writer.Key("world_points");
		writer.StartObject();
		for (int wp_idx = 0; wp_idx < configs->num_world_pts; wp_idx++) {
			writer.Key(std::to_string(wp_idx).c_str());
			writer.StartObject();
			{
				writer.Key("label");
				writer.Uint(prob.world_pt_label(wp_idx));
				writer.Key("point");
				writer.StartArray();
				writer.Double(prob.world_points(wp_idx)[0]);
				writer.Double(prob.world_points(wp_idx)[1]);
				writer.Double(prob.world_points(wp_idx)[2]);
				writer.EndArray();
			}
			writer.EndObject();
		}
		writer.EndObject();

		writer.Key("image_points");
		writer.StartArray();
		for (int obsv_idx = 0; obsv_idx < configs->num_observations; obsv_idx++) {
			writer.StartObject();
			writer.Key("wp_idx");
			writer.Uint(prob.obsv2worldpts_index(obsv_idx));
			writer.Key("label");
			writer.Uint(prob.world_pt_label(prob.obsv2worldpts_index(obsv_idx)));
			writer.Key("cam_idx");
			writer.Uint(prob.camera_index(obsv_idx));
			writer.Key("point");
			writer.StartArray();
			writer.Double(prob.image_points(obsv_idx)[0]);
			writer.Double(prob.image_points(obsv_idx)[1]);
			writer.EndArray();
			writer.EndObject();
		}
		writer.EndArray();
	}
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

	cout << endl << ">> Triangulation complete!" << endl;
	cout << "  input:" << input_image_pt_path.c_str() << endl;
	cout << "  saved:" << json_out_path.c_str() << endl;
	return 0;
}