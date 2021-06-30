<h1>Build bundle adjustment project (C/C++)</h1>

<h2>1. Windows</h2>

---

1. Modify CMakeList.txt

    From *./ceres_bundle_adjustment/CMakeLists.txt*:

           ...
        18 if (WIN32)
        19     # copy dlls
        20     set(CERES_ROOT_DIR "{root}/ceres")
           ...
        34 endif()
           ...

    change *{root}* to your own (e.g., *C:/Users/hjoon*) ceres root directory as explained in the [installation guide](./install_windows.md).

2. Run CMake

    Install CMake if not already installed: https://cmake.org/install/

    Navigate to *./ceres_bundle_adjustment*, make *build* directory, and run cmake

        mkdir build
        cd build
        cmake ..

3. Compile and build

    Once CMake build is complete, open the generated *./ceres_bundle_adjustment/build/CeresMulticamCalib.sln* in Visual Studio and build the project as *Release, x64*:

    <img src="./assets/vs_build.png" style="width:600px"/>

    This will generate *CeresMulticamCalib.exe* inside *./ceres_bundle_adjustment/build/bin/Release/*:

    <img src="./assets/vs_build2.png" style="width:500px"/>

(cf) Running *CeresMulticamCalib.exe* will look something like this:

<img src="./assets/bundle_adjustment_exe.png" style="width:800px"/>

You do not need to directly run *CeresMulticamCalib.exe*. Python codes will run it as explained in the *section II.* of [this tutorial](tutorial.md).

<h2>2. Linux</h2>

---

*Coming...*