// the configured options and settings
#pragma once
#define CeresMulticamCalib_VERSION_MAJOR 1
#define CeresMulticamCalib_VERSION_MINOR 0

#define ROOT_DIR "C:/Users/hjoon/OneDrive - University of Utah/HJ/PhD/Research/190817_FullBodyCapture/Codes/210410_MultiCamCalib/python/ceres_bundle_adjustment"
#define NUM_CAM_PARAMS 15
#define TEST "@CERES_INCLUDE_DIRS"

// check platform
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    // Windows (32-bit and 64-bit, this part is common)
    #define OS_WINDOWS
    #define READ_MODE "rb"
    #define WRITE_MODE "wb"

    #include <direct.h>
    #ifdef _WIN64
    // Windows (64-bit only)
    #else
    // Windows (32-bit only)
    #endif
#elif __APPLE__
    #include <TargetConditionals.h>
    #if TARGET_IPHONE_SIMULATOR
         // iOS Simulator
    #elif TARGET_OS_IPHONE
        // iOS device
    #elif TARGET_OS_MAC
        // Other kinds of Mac OS
        #define OS_MACOS
        #define READ_MODE "r"
        #define WRITE_MODE "w"
        #include <sys/stat.h>
    #else
    #   error "Unknown Apple platform"
    #endif
#elif __linux__
    // linux
    #define OS_LINUX
    #define READ_MODE "r"
    #define WRITE_MODE "w"
    #include <sys/stat.h>

#elif __unix__ // all unices not caught above
    // Unix
#elif defined(_POSIX_VERSION)
    // POSIX
#else
#   error "Unknown compiler"
#endif

// for concatenating filepaths
#if defined(WIN32)
#define OS_SEP "\\"
#else
#define OS_SEP "/"
#endif

#include <string>
struct Config {
    int max_iter, num_thread;
    double function_tolerance, parameter_tolerance, gradient_tolerance, inner_iteration_tolerance;
    int chb_n_rows, chb_n_cols, chb_sqr_size;
    int n_cams;

    std::string dir_output, dir_cam_params, dir_corners, dir_outliers, dir_world_points, dir_ceres_output;
};
