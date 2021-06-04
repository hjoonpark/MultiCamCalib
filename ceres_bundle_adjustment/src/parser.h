#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <array>

#include <rapidjson/rapidjson.h>
#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/filewritestream.h>

#include "Config.h"
#include "camera.h"
#include "frame.h"
#include "checkerboard.h"
#define MAX_BUF 65536

namespace Parser {
    rapidjson::Document readJson(const char*path) {
        FILE *fp;
        errno_t err = fopen_s(&fp, path, READ_MODE);
        
        char read_buf[MAX_BUF];
        rapidjson::FileReadStream fs(fp, read_buf, sizeof(read_buf));

        rapidjson::Document doc;
        doc.ParseStream(fs);

        return doc;
    }
    void writeJson(const char* out_path, const rapidjson::StringBuffer &writer_buf) {
        rapidjson::Document doc;
        doc.Parse(writer_buf.GetString());

        FILE* fp;
        errno_t err = fopen_s(&fp, out_path, WRITE_MODE);
        
        char json_buf[MAX_BUF];
        rapidjson::FileWriteStream json_os(fp, json_buf, sizeof(json_buf));
        rapidjson::PrettyWriter<rapidjson::FileWriteStream> writer_fs(json_os);
        doc.Accept(writer_fs);
        fclose(fp);
    }
    void loadConfig(const char* config_path, Config &config) {
        rapidjson::Document doc = Parser::readJson(config_path);

        // parse paths
        const rapidjson::Value &paths = doc["paths"];

        config.dir_output = ROOT_DIR + std::string(OS_SEP) + paths["output_dir"].GetString();
        config.dir_cam_params = config.dir_output + std::string(OS_SEP) + paths["cam_params"].GetString();
        config.dir_corners = config.dir_output + std::string(OS_SEP) + paths["corners"].GetString();
        config.dir_outliers = config.dir_output + std::string(OS_SEP) + paths["outliers"].GetString();
        config.dir_world_points = config.dir_output + std::string(OS_SEP) + paths["world_points"].GetString();
        config.dir_ceres_output = config.dir_output + std::string(OS_SEP) + paths["ceres_output"].GetString();

        // parse checkerboard info
        const rapidjson::Value &chb_info = doc["checkerboard"];
        config.chb_n_rows = chb_info["n_rows"].GetInt();
        config.chb_n_cols = chb_info["n_cols"].GetInt();
        config.chb_sqr_size = chb_info["sqr_size"].GetDouble(); // in milimeters

        // parse camera info
        const rapidjson::Value &cams_info = doc["cameras"];
        config.n_cams = cams_info["n_cams"].GetInt();

        // parse bundle adjustment configs
        const rapidjson::Value &bund_info = doc["bundle_adjustment"];
        config.max_iter = bund_info["max_iter"].GetInt();
        config.num_thread = bund_info["num_thread"].GetInt();
        config.function_tolerance = bund_info["function_tolerance"].GetDouble();
        config.parameter_tolerance = bund_info["parameter_tolerance"].GetDouble();
        config.gradient_tolerance = bund_info["gradient_tolerance"].GetDouble();
        config.inner_iteration_tolerance = bund_info["inner_iteration_tolerance"].GetDouble();
    }

    void loadInitialCamParams(const char* cam_path, const int n_cams, std::vector<Camera> &cameras) {
        rapidjson::Document doc = Parser::readJson(cam_path);

        cameras.clear();
        cameras.reserve(n_cams);
        for (int cam_idx = 0; cam_idx < n_cams; cam_idx++) {
            const rapidjson::Value &p = doc[std::to_string(cam_idx).c_str()];
            Camera *cam = new Camera(cam_idx, NUM_CAM_PARAMS);
            double fx = p["fx"].GetDouble();
            double fy = p["fy"].GetDouble();
            double cx = p["cx"].GetDouble();
            double cy = p["cy"].GetDouble();
            double k1 = p["k1"].GetDouble();
            double k2 = p["k2"].GetDouble();
            double p1 = p["p1"].GetDouble();
            double p2 = p["p2"].GetDouble();
            double k3 = p["k3"].GetDouble();

            const rapidjson::Value &rvec_ = p["rvec"];
            double rvec[3] = {rvec_[0].GetDouble(), rvec_[1].GetDouble(), rvec_[2].GetDouble()};
            const rapidjson::Value &tvec_ = p["tvec"];
            double tvec[3] = {tvec_[0].GetDouble(), tvec_[1].GetDouble(), tvec_[2].GetDouble()};

            cam->setCameraParameters(rvec, tvec, fx, fy, cx, cy, k1, k2, p1, p2, k3);
            cameras.push_back(*cam);
        }
    }

    void loadDetectionResult(const char* outlier_path, const char* detection_result_path, std::vector<Frame> &frames, const int n_cams) {
        // load outliers
        std::vector<std::string> outliers;
        if (outlier_path != NULL) {
            rapidjson::Document doc_outliers = Parser::readJson(outlier_path);
            for (auto key = doc_outliers.MemberBegin(); key != doc_outliers.MemberEnd(); key++) {
                const char* corner_id = key->name.GetString();
                const rapidjson::Value &res = doc_outliers[corner_id];
                std::string cam_img_name = res["img_name"].GetString();

                // push_back if not exists
                if (std::find(outliers.begin(), outliers.end(), cam_img_name) == outliers.end()) outliers.push_back(cam_img_name);
            }
        }
        // sort for fast look-up
        std::sort(outliers.begin(), outliers.end());
        std::vector<std::string>img_names2;
        rapidjson::Document doc = Parser::readJson(detection_result_path);
        frames.clear();
        const rapidjson::Value& det = doc["detections"];
        int outlier_idx = 0;
        std::cout << ">> Skipping images with outlier corners:" << std::endl;
        for (auto f = det.MemberBegin(); f != det.MemberEnd(); f++) {
            std::string img_name = f->name.GetString();
            
            Frame *frame = new Frame(img_name.c_str(), n_cams);
            const rapidjson::Value &res = det[img_name.c_str()];

            for(int cam_idx = 0; cam_idx < n_cams; cam_idx++) {
                std::string cam_img_name = std::to_string(cam_idx) + "_" + img_name;
                int detected = res[std::to_string(cam_idx).c_str()].GetInt();

                // mark as undetected if it's an outlier
                bool is_outlier = std::binary_search(outliers.begin(), outliers.end(), cam_img_name);
                if (is_outlier) {
                    outlier_idx += 1;
                    detected = 0;
                    img_names2.push_back(cam_img_name);
                    std::cout << cam_img_name << "  ";
                }
                frame->setDetected(cam_idx, (bool) detected);
            }

            if (frame->n_detected > 2) {
                frames.push_back(*frame);
            }
        }
        std::cout << std::endl << ">> " << outlier_idx << " images skipped." << std::endl << std::endl;
        std::sort(img_names2.begin(), img_names2.end());
    }

    void loadInitialCheckerboardPoses(const char* json_path, const std::vector<Frame> &frames, std::vector<Checkerboard> &checkerboards) {
        rapidjson::Document doc = Parser::readJson(json_path);

        checkerboards.clear();
        checkerboards.reserve(frames.size());
        for(int frame_idx = 0; frame_idx < frames.size(); frame_idx++) {
            Checkerboard *chb = new Checkerboard(frame_idx, frames[frame_idx].img_name);

            const char* img_name = frames[frame_idx].img_name.c_str();

            const rapidjson::Value &frame = doc["frames"];
            const rapidjson::Value &chb_data = frame[img_name];
            const rapidjson::Value &rvec = chb_data["rvec"];
            const rapidjson::Value &tvec = chb_data["tvec"];

            chb->rvec[0] = rvec[0].GetDouble();
            chb->rvec[1] = rvec[1].GetDouble();
            chb->rvec[2] = rvec[2].GetDouble();

            chb->tvec[0] = tvec[0].GetDouble();
            chb->tvec[1] = tvec[1].GetDouble();
            chb->tvec[2] = tvec[2].GetDouble();
            checkerboards.push_back(*chb);
        }
    }
    int loadImagePoints(const char* txt_path, double* img_pts) {
        std::ifstream file(txt_path);
        std::string str;

        int line_idx = 0;
        while(std::getline(file, str)) {
            std::vector<std::string> v;
            std::istringstream iss(str);
            std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(v));
            
            if (line_idx == 0) {
                int det = std::stoi(v[0]);
                if (det != 1) {
                    return -1;
                }
            } else {
                for (int i =0; i < v.size(); i++) {
                    img_pts[2*(line_idx-1) + i] = std::stod(v[i]);
                }
            }
            line_idx += 1;
        }
        return 0;
    }

    void saveBundleAdjustmentResult(const char* save_path, const double initial_cost, const double final_cost, const int n_iterations, const char* final_cam_param_path, const char* final_world_points_path) {
        rapidjson::StringBuffer writer_buf;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(writer_buf);
        writer.StartObject();
        {
            writer.Key("initial_cost");
            writer.Double(initial_cost);
            writer.Key("final_cost");
            writer.Double(final_cost);
            writer.Key("n_iterations");
            writer.Int(n_iterations);
            writer.Key("final_camera_parameters");
            writer.String(final_cam_param_path);
            writer.Key("final_world_points");
            writer.String(final_world_points_path);
        }
        writer.EndObject();
        Parser::writeJson(save_path, writer_buf);
    }
    void saveFinalCameraParameters(const char* save_path, std::vector<Camera> &cameras) {
        rapidjson::StringBuffer writer_buf;
        rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(writer_buf);
        writer.StartObject();
        for (int cam_idx = 0; cam_idx < cameras.size(); cam_idx++ ){
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
        
        Parser::writeJson(save_path, writer_buf);
    }

    void saveFinalWorldPoints(const char* save_path, std::vector<Checkerboard> &checkerboards, std::vector<Frame> &frames) {
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
        Parser::writeJson(save_path, writer_buf_wp);
    }
};