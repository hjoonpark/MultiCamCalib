#include "parser.h"
#include "worker.h"
using std::cout;
using std::endl;

Parser::Parser() {

}
int Parser::LoadInitialWorldPoints(const char* path, const int num_cams, const int num_corners, double* &wps_out) {
	std::ifstream file(path);
	std::string str;
	int line_idx = -1;
	int num_frames = 0;
	int frame_idx = 0;
	while (std::getline(file, str))
	{
		line_idx += 1;

		if (line_idx == 0) {
			std::vector<std::string> v;
			std::istringstream iss(str);
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(v));
			num_frames = std::stoi(v[0]);
			wps_out = new double[num_frames * num_corners * 3];
			continue;
		}

		if (line_idx >= num_frames + 3) {
			std::vector<std::string> v;
			std::istringstream iss(str);
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(v));
			
			for (int p_idx = 0; p_idx < num_corners; p_idx++) {
				double p[3] = { std::stod(v[2 + num_cams + p_idx * 3 + 0]), std::stod(v[2 + num_cams + p_idx * 3 + 1]), std::stod(v[2 + num_cams + p_idx * 3 + 2]) };

				wps_out[frame_idx * num_corners * 3 + p_idx * 3 + 0] = p[0];
				wps_out[frame_idx * num_corners * 3 + p_idx * 3 + 1] = p[1];
				wps_out[frame_idx * num_corners * 3 + p_idx * 3 + 2] = p[2];
			}
			frame_idx += 1;
			if (frame_idx == num_frames) {
				break;
			}
		}
	}
	return 0;
}
int Parser::WriteCamParamsToJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer, const int cam_idx, const double *cp) {
	writer.Key(std::to_string(cam_idx).c_str());

	writer.StartObject();
	writer.Key("rvec");
	writer.StartArray();
	writer.Double(cp[0]);
	writer.Double(cp[1]);
	writer.Double(cp[2]);
	writer.EndArray();

	writer.Key("tvec");
	writer.StartArray();
	writer.Double(cp[3]);
	writer.Double(cp[4]);
	writer.Double(cp[5]);
	writer.EndArray();

	writer.Key("fx");
	writer.Double(cp[6]);
	writer.Key("fy");
	writer.Double(cp[7]);
	writer.Key("cx");
	writer.Double(cp[8]);
	writer.Key("cy");
	writer.Double(cp[9]);

	writer.Key("k1");
	writer.Double(cp[10]);
	writer.Key("k2");
	writer.Double(cp[11]);
	writer.Key("k3");
	writer.Double(cp[12]);
	writer.Key("k4");
	writer.Double(cp[13]);
	writer.Key("k5");
	writer.Double(cp[14]);
	writer.Key("k6");
	writer.Double(cp[15]);

	writer.Key("p1");
	writer.Double(cp[16]);
	writer.Key("p2");
	writer.Double(cp[17]);
	writer.Key("p3");
	writer.Double(cp[18]);
	writer.Key("p4");
	writer.Double(cp[19]);
	writer.Key("p5");
	writer.Double(cp[20]);
	writer.Key("p6");
	writer.Double(cp[21]);

	writer.EndObject();
	return 0;
}

int Parser::WriteCamParamsToJson(rapidjson::PrettyWriter<rapidjson::StringBuffer>& writer, const int cam_idx, const Camera *camera) {
	writer.Key(std::to_string(cam_idx).c_str());

	writer.StartObject();
	writer.Key("rvec");
	writer.StartArray();
	writer.Double(camera->rvec[0]);
	writer.Double(camera->rvec[1]);
	writer.Double(camera->rvec[2]);
	writer.EndArray();

	writer.Key("tvec");
	writer.StartArray();
	writer.Double(camera->tvec[0]);
	writer.Double(camera->tvec[1]);
	writer.Double(camera->tvec[2]);
	writer.EndArray();

	writer.Key("fx");
	writer.Double(camera->fx);
	writer.Key("fy");
	writer.Double(camera->fy);
	writer.Key("cx");
	writer.Double(camera->cx);
	writer.Key("cy");
	writer.Double(camera->cy);

	writer.Key("k1");
	writer.Double(camera->k[0]);
	writer.Key("k2");
	writer.Double(camera->k[1]);
	writer.Key("k3");
	writer.Double(camera->k[2]);
	writer.Key("k4");
	writer.Double(camera->k[3]);
	writer.Key("k5");
	writer.Double(camera->k[4]);
	writer.Key("k6");
	writer.Double(camera->k[5]);

	writer.Key("p1");
	writer.Double(camera->p[0]);
	writer.Key("p2");
	writer.Double(camera->p[1]);
	writer.Key("p3");
	writer.Double(camera->p[2]);
	writer.Key("p4");
	writer.Double(camera->p[3]);
	writer.Key("p5");
	writer.Double(camera->p[4]);
	writer.Key("p6");
	writer.Double(camera->p[5]);

	writer.EndObject();
	return 0;
}

int Parser::LoadImagePoints(const char* path, Configs &configs_out, bool *&detected_out, double *&img_pts_out, std::vector<std::string> &img_names_vec_out, const int sanity_num_corners) {
	printf("Load json: %s\n", path);
	FILE* fp = fopen(path, "rb"); // non-Windows use "r"

	char readBuffer[65536]; // max buffer size
	rapidjson::FileReadStream fs(fp, readBuffer, sizeof(readBuffer));

	// parse
	rapidjson::Document doc;
	doc.ParseStream(fs);
	assert(doc.HasMember("configs"));
	assert(doc.HasMember("frames"));


	// configs
	const rapidjson::Value& configs = doc["configs"];
	const rapidjson::Value& chb = configs["chb"];
	configs_out.num_cams = configs["num_cams"].GetInt();
	configs_out.num_frames = configs["num_frames"].GetInt();
	configs_out.chb_num_corners = chb["num_corners"].GetInt();
	configs_out.chb_num_rows = chb["num_rows"].GetInt();
	configs_out.chb_num_cols = chb["num_cols"].GetInt();
	configs_out.chb_sqr_size = chb["chb_sqr_size"].GetInt();

	// sanity check
	/*
	if (configs_out.chb_num_corners != sanity_num_corners) {
		std::cerr << "[ERROR] some mismatch!" << endl;
		std::getchar();
		return 1;
	}
	*/

	detected_out = new bool[configs_out.num_frames * configs_out.num_cams];
	img_pts_out = new double[configs_out.num_frames * configs_out.num_cams * configs_out.chb_num_corners * 2]{ 0 };
	const rapidjson::Value& frames = doc["frames"];
	int num_img_pts = 0;
	int f_idx = 0;
	for (rapidjson::SizeType frame_idx = 0; frame_idx < frames.Size(); frame_idx++) {
		const rapidjson::Value& frame = frames[frame_idx];
		const rapidjson::Value& img_pts = frame["img_pts"];
		std::string img_name = frame["img_name"].GetString();
		// image points
		int num_detected = 0;
		for (int cam_idx = 0; cam_idx < configs_out.num_cams; cam_idx++) {
			std::string cam_idx_str = std::to_string(cam_idx);
			if (img_pts.HasMember(cam_idx_str.c_str())) {
				detected_out[frame_idx * configs_out.num_cams + cam_idx] = true;
				num_detected += 1;
			}
			else
			{
				detected_out[frame_idx * configs_out.num_cams + cam_idx] = false;
			}
		}

		if (num_detected >= 2) {
			img_names_vec_out.push_back(img_name);
			for (int cam_idx = 0; cam_idx < configs_out.num_cams; cam_idx++) {
				std::string cam_idx_str = std::to_string(cam_idx);
				if (img_pts.HasMember(cam_idx_str.c_str())) {
					const rapidjson::Value &pts = img_pts[cam_idx_str.c_str()];
					for (int p_idx = 0; p_idx < configs_out.chb_num_corners; p_idx++) {
						const rapidjson::Value &pt = pts[p_idx];
						double u = pt[0].GetDouble();
						double v = pt[1].GetDouble();
						int idx = frame_idx * configs_out.num_cams * configs_out.chb_num_corners * 2 + cam_idx * configs_out.chb_num_corners * 2 + p_idx * 2;
						img_pts_out[idx + 0] = u;
						img_pts_out[idx + 1] = v;
						num_img_pts += 1;
					}
				}
			}
		}
		else {
			cout << "num_detected = " << num_detected <<", skipping:" << img_name << endl;
		}
	}
	fclose(fp);
	std::sort(img_names_vec_out.begin(), img_names_vec_out.end());
	printf("  %d frames, %d image points loaded.\n", frames.Size(), num_img_pts);
	return 0;
}
int Parser::LoadInitialParameters(const char* path, const std::vector<std::string> &img_names_vec_in, Configs &configs_out, BundleAdjParameters& params) {
	printf("Load json: %s\n", path);
	FILE* fp = fopen(path, "rb"); // non-Windows use "r"

	char readBuffer[65536]; // max buffer size
	rapidjson::FileReadStream fs(fp, readBuffer, sizeof(readBuffer));

	// parse
	rapidjson::Document doc;
	doc.ParseStream(fs);
	assert(doc.HasMember("configs"));
	assert(doc.HasMember("cam_params"));
	assert(doc.HasMember("chb"));

	const rapidjson::Value& configs = doc["configs"];
	int num_cams = configs["num_cams"].GetInt();
	configs_out.num_cam_params = configs["num_cam_params"].GetInt();

	int num_corners = configs["num_corners"].GetInt();
	int num_frames = img_names_vec_in.size();

	const rapidjson::Value& cam_params = doc["cam_params"];
	const rapidjson::Value& chb_params = doc["chb"];
	params.SetCamParams(num_cams, configs_out.num_cam_params, cam_params);
	params.SetChbParams(img_names_vec_in, num_frames, num_corners, chb_params);
	cout << "Loaded." << endl;
	return 0;
}
