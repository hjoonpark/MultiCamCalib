#pragma once

class Checkerboard {
public:
    int idx;
    std::string img_name;
    double rvec[3];
    double tvec[3];

    static size_t n_rows, n_cols, n_pts;
    static double sqr_size;
    static double *chb_pts;

    Checkerboard(const int idx_, const std::string img_name_) {
        idx = idx_;
        img_name = img_name_;
    }
    
    static void initGlobalProperties(size_t n_rows_, size_t n_cols_, double sqr_size_) {
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
        n_pts = n_rows*n_cols;
        chb_pts = new double[3*n_pts];
        for (size_t r = 0; r < n_rows; r++ ) {
            for (size_t c = 0; c < n_cols; c++) {
                size_t i = r*n_cols + c;
                chb_pts[3*i] = r*sqr_size;
                chb_pts[3*i + 1] = c*sqr_size;
                chb_pts[3*i + 2] = 0;
            }
        }
    }
    ~Checkerboard() {
    }
};

size_t Checkerboard::n_rows;
size_t Checkerboard::n_cols;
size_t Checkerboard::n_pts;
double Checkerboard::sqr_size;
double* Checkerboard::chb_pts;