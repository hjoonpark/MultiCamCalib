cmake_minimum_required(VERSION 3.0)

# set the project name and version
project(CeresMulticamCalib VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

IF(MSVC_VERSION EQUAL 1400 OR MSVC_VERSION GREATER 1400)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE -D_CRT_NONSTDC_NO_DEPRECATE
    -D_CRT_SECURE_NO_WARNINGS)
    ADD_DEFINITIONS(-D_SCL_SECURE_NO_DEPRECATE -D_SCL_SECURE_NO_WARNINGS)
ENDIF(MSVC_VERSION EQUAL 1400 OR MSVC_VERSION GREATER 1400)

configure_file(src/Config.h.in ./../src/Config.h)

set(TARGET_NAME CeresMulticamCalib)

add_executable(${TARGET_NAME} src/main.cpp)

target_include_directories(${TARGET_NAME} PUBLIC
                            "${PROJECT_BINARY_DIR}"
                            )
include_directories("include")

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
message("-------------------")
message( ${CMAKE_CURRENT_SOURCE_DIR} )
message(${PROJECT_BINARY_DIR})
message(${CERES_LIBRARIES})
message(${RUNTIME_OUTPUT_DIRECTORY})
message("-------------------")