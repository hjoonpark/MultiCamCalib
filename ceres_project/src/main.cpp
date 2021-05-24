// A simple program that computes the square root of a number
#define _CRT_SECURE_NO_WARNINGS

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "Config.h"
#include "parser.h"
#include "checkerboard.h"
#include "bund_adj_6dof.h"
int main(int argc, char* argv[])
{
    std::cout << argv[0] << " Version " << CeresMulticamCalib_VERSION_MAJOR << "." << CeresMulticamCalib_VERSION_MINOR << std::endl;

    std::stringstream config_path;
    config_path << ROOT_DIR << "config.json";
    int n_rows, n_cols, n_cams;
    float sqr_size;
    std::string output_dir;
    Parser::loadConfig(config_path.str().c_str(), output_dir, n_rows, n_cols, sqr_size, n_cams);
    std::cout << "Read: " << config_path.str() << std::endl;
    std::cout << "  - chb=(" << n_rows << " x " << n_cols << "), size=" << sqr_size << "mm" << ", n_cams=" << n_cams << ", output_dir=" << output_dir.c_str() << std::endl;

    BundAdj6Dof task;
    task.init(output_dir.c_str(), n_cams, n_rows, n_cols, sqr_size);
    task.run(output_dir.c_str());

    int i;
    std::cin >> i;
    return 0;
}
