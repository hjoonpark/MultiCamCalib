#pragma once
#include <iostream>
struct Camera {
    int idx;
    double *params;
    Camera(const int idx_, const int n_params) {
        idx = idx_;
        params = new double[n_params];
    }
    ~Camera() {
        // std::cout << "!! deleted camera " << idx << std::endl;
    }
    void setCameraParameters(const double (&rvec)[3], const double (&tvec)[3], 
                            const double fx, const double fy, const double cx, const double cy,
                            const double k1, const double k2, const double p1, const double p2, const double k3) {
        params[0] = rvec[0];
        params[1] = rvec[1];
        params[2] = rvec[2];

        params[3] = tvec[0];
        params[4] = tvec[1];
        params[5] = tvec[2];

        params[6] = fx;
        params[7] = fy;
        params[8] = cx;
        params[9] = cy;

        params[10] = k1;
        params[11] = k2;
        params[12] = p1;
        params[13] = p2;
        params[14] = k3;
    } 
};