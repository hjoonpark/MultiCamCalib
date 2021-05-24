cmake_minimum_required(VERSION 3.0)

# set the project name and version
project(CeresMulticamCalib VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

configure_file(src/Config.h.in ./../src/Config.h)

add_executable(CeresMulticamCalib src/main.cpp)

target_include_directories(CeresMulticamCalib PUBLIC
                            "${PROJECT_BINARY_DIR}"
                            )
include_directories("include")

# link external ceres library
# find_package(Ceres REQUIRED)
find_package(Ceres REQUIRED PATHS "C:/Users/hjoon/ceres")

target_include_directories(CeresMulticamCalib
    PRIVATE
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(CeresMulticamCalib
    PRIVATE
    ${CERES_LIBRARIES}
)


message("-------------------")
message( ${CMAKE_CURRENT_SOURCE_DIR} )
message(${PROJECT_BINARY_DIR})
message(${CERES_LIBRARIES})
message("-------------------")
