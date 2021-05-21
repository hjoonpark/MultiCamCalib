// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "Config.h"
#include "bund_adj_6dof.h"

int main(int argc, char* argv[])
{
    int i;
    if (argc < 2) {   
        std::cout << argv[0] << " Version " << CeresMulticamCalib_VERSION_MAJOR << "." << CeresMulticamCalib_VERSION_MINOR << std::endl;
        std::cin >> i;
        return 1;
    }

    std::cin >> i;
    return 0;
}
