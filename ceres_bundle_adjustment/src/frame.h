#pragma once
#include <string>
#include <iostream>
struct Frame {
    std::string img_name;
    bool *detected;
    int n_detected;
    Frame(const std::string &img_name_, const int n_cams) {
        img_name = img_name_;
        detected = new bool[n_cams];
        n_detected = 0;
    }
    ~Frame () {
        // std::cout << "!! frame delete" << std::endl;
    }
    void setDetected(const int cam_idx, const bool res) {
        detected[cam_idx] = res;
        if (res) n_detected += 1;
    }
};