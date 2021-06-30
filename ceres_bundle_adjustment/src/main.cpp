#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdlib.h>
#include <string>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "config.h"
#include "parser.h"
#include "checkerboard.h"
#include "bund_adj_6dof.h"
int main(int argc, char* argv[])
{
    std::cout << argv[0] << " Version " << CeresMulticamCalib_VERSION_MAJOR << "." << CeresMulticamCalib_VERSION_MINOR << std::endl;

    std::stringstream config_path;
    config_path << ROOT_DIR <<  OS_SEP << ".." << OS_SEP << "multicamcalib" << OS_SEP << "config.json";
    Config config;
    Parser::loadConfig(config_path.str().c_str(), config);
    std::cout << "Read: " << config_path.str() << std::endl;
    std::cout << "  - chb=(" << config.chb_n_rows << " x " << config.chb_n_cols << "), size=" << config.chb_sqr_size << "mm" << ", n_cams=" << config.n_cams << std::endl;

    // create output directory
    #ifdef OS_WINDOWS
        int mkdir_res = _mkdir(config.dir_ceres_output.c_str());
    #else
        int mkdir_res = mkdir(config.dir_ceres_output.c_str(), 0755);
    #endif

    if (mkdir_res != 0)
    {
        std::cerr << "Failed to create output directory: " << config.dir_ceres_output << std::endl;
    } else{
        std::cout << "Created output directory: " << config.dir_ceres_output << std::endl;
    }

    BundAdj6Dof task;
    task.init(config);
    task.run(config);

    return 0;
}
