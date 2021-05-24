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

#define MAX_BUF 65536
namespace Parser {
    rapidjson::Document readJson(const char*path) {
        FILE *fp;
        fp = fopen(path, READ_MODE);
        
        char read_buf[MAX_BUF];
        rapidjson::FileReadStream fs(fp, read_buf, sizeof(read_buf));

        rapidjson::Document doc;
        doc.ParseStream(fs);

        return doc;
    }
    void loadConfig(const char* config_path, std::string &output_dir, int &n_rows, int &n_cols, float &sqr_size, int &n_cams) {
        rapidjson::Document doc = Parser::readJson(config_path);

        // parse paths
        output_dir = doc["output_dir"].GetString();

        // parse checkerboard info
        const rapidjson::Value &chb_info = doc["checkerboard"];
        n_rows = chb_info["n_rows"].GetInt();
        n_cols = chb_info["n_cols"].GetInt();
        sqr_size = chb_info["sqr_size"].GetFloat(); // in milimeters

        // parse camera info
        const rapidjson::Value &cams_info = doc["cameras"];
        n_cams = cams_info["n_cams"].GetInt();
    }

    void loadInitialCamParams(const char* cam_path, const int n_cams, std::vector<Camera> &cameras) {
        rapidjson::Document doc = Parser::readJson(cam_path);

        cameras.clear();
        cameras.reserve(n_cams);
        for (int cam_idx = 0; cam_idx < n_cams; cam_idx++) {
            const rapidjson::Value &p = doc[std::to_string(cam_idx).c_str()];
            Camera cam(cam_idx, NUM_CAM_PARAMS);
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

            cam.setCameraParameters(rvec, tvec, fx, fy, cx, cy, k1, k2, p1, p2, k3);
            cameras.push_back(cam);
        }
    }

    void loadDetectionResult(const char* json_path, std::vector<Frame> &frames, const int n_cams) {
        rapidjson::Document doc = Parser::readJson(json_path);

        frames.clear();
        const rapidjson::Value& det = doc["detections"];
        for (auto f = det.MemberBegin(); f != det.MemberEnd(); f++) {
            std::string img_name = f->name.GetString();
            
            Frame frame(img_name.c_str(), n_cams);
            const rapidjson::Value &res = det[img_name.c_str()];

            for(int cam_idx = 0; cam_idx < n_cams; cam_idx++) {
                int detected = res[std::to_string(cam_idx).c_str()].GetInt();
                frame.setDetected(cam_idx, (bool) detected);
            }
            if (frame.n_detected > 2) {
                frames.push_back(frame);
            }
        }
    }

    void loadInitialCheckerboardPoses(const char* json_path, const std::vector<Frame> &frames, std::vector<std::array<double, 6>> &chb_poses) {
        rapidjson::Document doc = Parser::readJson(json_path);

        chb_poses.clear();
        chb_poses.reserve(frames.size());
        for(int frame_idx = 0; frame_idx < frames.size(); frame_idx++) {
            const char* img_name = frames[frame_idx].img_name.c_str();

            const rapidjson::Value &chb_data = doc[img_name];
            const rapidjson::Value &rvec = chb_data["rvec"];
            const rapidjson::Value &tvec = chb_data["tvec"];

            std::array<double, 6> pose = {rvec[0].GetDouble(), rvec[1].GetDouble(), rvec[2].GetDouble(), tvec[0].GetDouble(), tvec[1].GetDouble(), tvec[2].GetDouble()};
            chb_poses.push_back(pose);
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
};