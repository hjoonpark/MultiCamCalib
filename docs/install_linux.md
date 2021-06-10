# Ceres installation - Linux (using Ubuntu 18.04.5 LTS)
## 1. Install all the dependencies following the [official guide](http://ceres-solver.org/installation.html).
    # CMake
    sudo apt-get install cmake

    # google-glog + gflags
    sudo apt-get install libgoogle-glog-dev libgflags-dev

    # BLAS & LAPACK
    sudo apt-get install libatlas-base-dev

    # Eigen3
    sudo apt-get install libeigen3-dev

    # SuiteSparse and CXSparse (optional)
    sudo apt-get install libsuitesparse-dev

## 2. Download ceres 

    git clone https://ceres-solver.googlesource.com/ceres-solver

## 3. Navigate to the cloned *ceres-solver* folder and cmake 
    
    cd {root}/ceres-solver
    mkdir build
    cd build
    cmake ..
    make -j3
    sudo make install

<b>For my PC, `{root}` is `~/Desktop`.