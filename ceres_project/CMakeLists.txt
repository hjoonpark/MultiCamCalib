cmake_minimum_required(VERSION 3.0)

# set the project name and version
project(CeresMulticamCalib VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(TARGET_NAME CeresMulticamCalib)
set(PROJECT_OUTPUT_DIR "${PROJECT_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_OUTPUT_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_OUTPUT_DIR})

set(SUITESPARSE_WIN_x64_DIR "C:/Users/hjoon/ceres/suitesparse/lapack_windows/x64")
set(PROJECT_OUTPUT_DIR_RELEASE "${PROJECT_OUTPUT_DIR}/Release")
set(DLL_PATHS 
    "${SUITESPARSE_WIN_x64_DIR}/libblas.dll"
    "${SUITESPARSE_WIN_x64_DIR}/liblapack.dll"
    "${SUITESPARSE_WIN_x64_DIR}/libgcc_s_sjlj-1.dll"
    "${SUITESPARSE_WIN_x64_DIR}/libgfortran-3.dll"
    "${SUITESPARSE_WIN_x64_DIR}/libquadmath-0.dll"
)

file(COPY ${DLL_PATHS}
    DESTINATION ${PROJECT_OUTPUT_DIR_RELEASE})

configure_file(src/Config.h.in ./../src/Config.h)


add_executable(${TARGET_NAME} src/main.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
                            "include"
                            )

# link external ceres library
# find_package(Ceres REQUIRED)
find_package(Ceres REQUIRED PATHS "C:/Users/hjoon/ceres/ceres-install")

target_include_directories(${TARGET_NAME}
    PRIVATE
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(${TARGET_NAME}
    PRIVATE
    ${CERES_LIBRARIES}
)