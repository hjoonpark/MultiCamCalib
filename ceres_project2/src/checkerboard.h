#pragma once

struct Checkerboard {
    size_t n_rows, n_cols, n_pts;
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
        n_pts = n_rows*n_cols;
        chb_pts = new double[3*n_pts];
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