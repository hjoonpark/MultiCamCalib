#pragma once
#include <iostream>

#define RVEC 0
#define TVEC 3
#define FX 6
#define FY 7
#define CX 8
#define CY 9
#define K1 10
#define K2 11
#define P1 12
#define P2 13
#define K3 14
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
        params[RVEC] = rvec[0];
        params[RVEC+1] = rvec[1];
        params[RVEC+2] = rvec[2];

        params[TVEC] = tvec[0];
        params[TVEC+1] = tvec[1];
        params[TVEC+2] = tvec[2];

        params[FX] = fx;
        params[FY] = fy;
        params[CX] = cx;
        params[CY] = cy;

        params[K1] = k1;
        params[K2] = k2;
        params[P1] = p1;
        params[P2] = p2;
        params[K3] = k3;
    }

    double* rvec() {
        return &params[RVEC];
    }
    double* tvec() {
        return &params[TVEC];
    }
    double& fx() {
        return params[FX];
    }
    double& fy() {
        return params[FY];
    }
    double& cx() {
        return params[CX];
    }
    double& cy() {
        return params[CY];
    }
    double& k1() {
        return params[K1];
    }
    double& k2() {
        return params[K2];
    }
    double& p2() {
        return params[P2];
    }
    double& p1() {
        return params[P1];
    }
    double& k3() {
        return params[K3];
    }
};